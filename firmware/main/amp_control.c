/* Optional amplifier I2C control. All state, JSON, NVS and bus work is core0.
 * There is no PCM hook, DSP descriptor change or core1 callback in this module. */
#include "amp_control.h"
#include "amp_profile.h"
#include "hw_mute.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG="amp_i2c";
enum { OK, BAD_LENGTH, BAD_VALUE, BUSY, STALE, SAVE_ERROR, IO_ERROR, NO_MEMORY };
enum { DISABLED, STARTING, READY, FAULT };
typedef struct {
    uint8_t state, muted; /* muted: 0/1 confirmed command, 2 unknown after IO fault */
    int32_t error;
    uint32_t writes, reads, jobs;
} port_status_t;
static SemaphoreHandle_t lock;
static amp_profile_t *pending;
static char *requested_json;
static uint16_t requested_len;
static uint32_t generation=1, applied_generation;
static bool applying, started, persisted, clocks_ready;
static port_status_t status[AMP_PORTS];
static uint8_t current_functions[AMP_PORTS];
static uint8_t run_port, run_function, run_token, last_token, last_result;
static bool run_pending;
static uint8_t *upload;
static uint16_t upload_size, upload_received;
static uint8_t upload_token;
static uint32_t upload_generation;
static uint64_t upload_session;
static int64_t upload_deadline;
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t devices[AMP_PORTS];
static unsigned io_port;
static void amp_task(void *unused);

static void take(void) { xSemaphoreTake(lock,portMAX_DELAY); }
static void give(void) { xSemaphoreGive(lock); }
static const char *json_text(void) { return requested_json?requested_json:amp_profile_default_json(); }
static bool needs_bus(const amp_profile_t *p)
{ return p->enabled && (p->ports[0].enabled||p->ports[1].enabled); }
/* Caller owns lock. The new task cannot inspect pending before that lock is
 * released. Nothing is allocated for an unused hardware-mode amplifier. */
static esp_err_t start_worker_locked(void)
{
    if(started||!clocks_ready) return ESP_OK;
    if(xTaskCreatePinnedToCore(amp_task,"amp_i2c",4096,NULL,1,NULL,0)!=pdPASS) return ESP_ERR_NO_MEM;
    started=true;
    return ESP_OK;
}
static uint16_t u16(const uint8_t *p) { return p[0]|((uint16_t)p[1]<<8); }
static uint32_t u32(const uint8_t *p) { return u16(p)|((uint32_t)u16(p+2)<<16); }
static void put32(uint8_t *p,uint32_t n) { for(int i=0;i<4;++i) p[i]=(uint8_t)(n>>(i*8)); }
static void discard_upload(void)
{
    free(upload); upload=NULL; upload_size=upload_received=0;
    upload_deadline=0; upload_session=0;
}
void amp_control_abort_upload(void) { if(lock) { take(); discard_upload(); give(); } }

static esp_err_t save_profile(const char *text,size_t n)
{
    nvs_handle_t h;
    esp_err_t err=nvs_open("amp_i2c",NVS_READWRITE,&h);
    if(err!=ESP_OK) return err;
    err=nvs_set_blob(h,"profile_v1",text,n);
    if(err==ESP_OK) err=nvs_commit(h);
    nvs_close(h);
    return err;
}

static int bus_write(void *context,uint8_t reg,const uint8_t *data,size_t n)
{
    (void)context;
    uint8_t bytes[AMP_BURST_MAX+1]; bytes[0]=reg; memcpy(bytes+1,data,n);
    esp_err_t err=i2c_master_transmit(devices[io_port],bytes,n+1,5);
    take(); if(err==ESP_OK) ++status[io_port].writes; give();
    return err;
}
static int bus_read(void *context,uint8_t reg,uint8_t *data,size_t n)
{
    (void)context;
    esp_err_t err=i2c_master_transmit_receive(devices[io_port],&reg,1,data,n,5);
    take(); if(err==ESP_OK) ++status[io_port].reads; give();
    return err;
}
static int execute(const amp_profile_t *cfg,unsigned port,unsigned function)
{
    const amp_port_profile_t *p=&cfg->ports[port];
    if(!devices[port]||function>=p->function_count) return ESP_ERR_INVALID_STATE;
    const amp_function_t *f=&p->functions[function];
    io_port=port;
    const amp_bus_io_t io={.write=bus_write,.read=bus_read};
    for(unsigned i=0;i<f->count;++i) {
        const amp_op_t *op=&p->ops[f->first+i];
        if(op->kind==AMP_DELAY) {
            /* Only this priority-1 core0 task waits, never the audio producers. */
            TickType_t ticks=pdMS_TO_TICKS(op->delay_ms);
            vTaskDelay(ticks?ticks:1);
        } else {
            const int err=amp_profile_op(op,&io);
            if(err) return err;
        }
        /* Let idle and protocol run between register operations as well. */
        vTaskDelay(1);
    }
    take(); ++status[port].jobs; give();
    return 0;
}
static void result(unsigned port,int err,uint8_t muted)
{
    take();
    status[port].error=err;
    status[port].state=err?FAULT:READY;
    status[port].muted=err?2:muted;
    give();
    if(err) ESP_LOGE(TAG,"port=%u error=%d mute_unknown=1 auto_retry=0",port,err);
}
static int close_bus(const amp_profile_t *cfg)
{
    int first=0;
    for(unsigned p=0;p<AMP_PORTS;++p) if(devices[p]) {
        /* Try the OLD profile's mute before removing a device. Failure is
         * reported; an absent bus/ACK must never become a claimed mute. */
        int err=execute(cfg,p,1);
        if(err&&!first) first=err;
        err=i2c_master_bus_rm_device(devices[p]);
        if(err&&!first) first=err;
        if(!err) devices[p]=NULL;
    }
    if(bus && !devices[0] && !devices[1]) {
        int err=i2c_del_master_bus(bus);
        if(err&&!first) first=err;
        if(!err) bus=NULL;
    }
    return first;
}
static int open_bus(const amp_profile_t *cfg)
{
    if(!needs_bus(cfg)) return 0;
    const i2c_master_bus_config_t bc={.i2c_port=-1,.sda_io_num=cfg->sda,.scl_io_num=cfg->scl,
        .clk_source=I2C_CLK_SRC_DEFAULT,.glitch_ignore_cnt=7,
        .flags.enable_internal_pullup=true};
    int err=i2c_new_master_bus(&bc,&bus);
    if(err) return err;
    for(unsigned p=0;p<AMP_PORTS;++p) if(cfg->ports[p].enabled) {
        const i2c_device_config_t dc={.dev_addr_length=I2C_ADDR_BIT_LEN_7,
            .device_address=cfg->ports[p].address,.scl_speed_hz=cfg->hz};
        err=i2c_master_bus_add_device(bus,&dc,&devices[p]);
        if(err) return err;
    }
    return 0;
}

static void amp_task(void *unused)
{
    (void)unused;
    amp_profile_t *active=NULL;
    for(;;) {
        amp_profile_t *next=NULL;
        uint32_t next_generation=0;
        take();
        if(pending) { next=pending; pending=NULL; next_generation=generation; applying=true; }
        give();
        if(next) {
            const int close_error=active?close_bus(active):0;
            /* If a driver handle failed to close, retain its profile and
             * report failure. Do not claim another pin/bus over live handles. */
            if(bus||devices[0]||devices[1]) {
                for(unsigned p=0;p<AMP_PORTS;++p) result(p,close_error?close_error:ESP_FAIL,2);
                free(next);
            } else {
                free(active); active=next;
                const int err=open_bus(active);
                for(unsigned p=0;p<AMP_PORTS;++p) {
                    take(); current_functions[p]=active->ports[p].function_count; give();
                    take(); status[p].state=active->enabled&&active->ports[p].enabled?STARTING:DISABLED;
                    status[p].muted=2; status[p].error=close_error; give();
                    if(active->enabled&&active->ports[p].enabled) {
                        int e=err?err:execute(active,p,0);
                        if(!e) e=execute(active,p,1); /* startup always ends muted */
                        result(p,e,1);
                    }
                }
                take(); applied_generation=next_generation; give();
                if(!needs_bus(active)) { free(active); active=NULL; }
            }
            take(); applying=false; give();
        }

        bool run=false; unsigned rp=0,rf=0; uint8_t token=0;
        take();
        if(run_pending) { run=true; rp=run_port; rf=run_function; token=run_token; run_pending=false; applying=true; }
        give();
        if(run) {
            int err=active?execute(active,rp,rf):ESP_ERR_INVALID_STATE;
            /* A custom function cannot leave the automatic PCM activity
             * policy overridden. Reassert the latest requested mute state. */
            const bool muted=hw_mute_is_muted();
            if(!err) err=execute(active,rp,muted?1:2);
            result(rp,err,muted);
            take(); last_token=token; last_result=err?IO_ERROR:OK; applying=false; give();
        }
        if(active) for(unsigned p=0;p<AMP_PORTS;++p) {
            take(); const bool ready=status[p].state==READY; const uint8_t was=status[p].muted; give();
            const bool muted=hw_mute_is_muted();
            if(ready && was!=muted) result(p,execute(active,p,muted?1:2),muted);
        }
        /* No bus polling or device discovery when disabled. PCM counters are
         * interpreted once by hw_mute: UAC OR BT, including digital silence. */
        take();
        const bool stop=!active&&!pending&&!run_pending;
        if(stop) started=false;
        give();
        if(stop) { vTaskDelete(NULL); return; }
        vTaskDelay(pdMS_TO_TICKS(10)?pdMS_TO_TICKS(10):1);
    }
}

esp_err_t amp_control_init(void)
{
    lock=xSemaphoreCreateMutex();
    if(!lock) return ESP_ERR_NO_MEM;
    const char *defaults=amp_profile_default_json();
    requested_len=(uint16_t)strlen(defaults);
    applied_generation=generation;
    for(unsigned p=0;p<AMP_PORTS;++p) status[p].muted=2;
    size_t n=0;
    nvs_handle_t h;
    esp_err_t err=nvs_open("amp_i2c",NVS_READONLY,&h);
    if(err!=ESP_OK) {
        if(err!=ESP_ERR_NVS_NOT_FOUND) last_result=SAVE_ERROR;
        return ESP_OK; /* Hardware-mode default: no heap or worker. */
    }
    err=nvs_get_blob(h,"profile_v1",NULL,&n);
    persisted=err==ESP_OK;
    if(err==ESP_OK && n && n<=AMP_JSON_MAX) {
        requested_json=calloc(1,n+1);pending=calloc(1,sizeof(*pending));
        if(requested_json&&pending) err=nvs_get_blob(h,"profile_v1",requested_json,&n);
        else err=ESP_ERR_NO_MEM;
    } else err=ESP_ERR_INVALID_SIZE;
    nvs_close(h);
    char error[96];
    if(err!=ESP_OK||!amp_profile_parse(requested_json,n,pending,error,sizeof(error))) {
        free(requested_json);free(pending);requested_json=NULL;pending=NULL;
        last_result=err==ESP_ERR_NO_MEM?NO_MEMORY:BAD_VALUE;
        ESP_LOGW(TAG,"saved amplifier profile not loaded; bus disabled; saved bytes retained");
        return ESP_OK;
    }
    requested_json[n]=0; requested_len=(uint16_t)n;
    if(!needs_bus(pending)) { free(pending);pending=NULL; }
    else applied_generation=0;
    return ESP_OK;
}
esp_err_t amp_control_start(void)
{
    if(!lock) return ESP_ERR_INVALID_STATE;
    take();clocks_ready=true;
    esp_err_t err=pending?start_worker_locked():ESP_OK;
    give();
    ESP_LOGI(TAG,"core0 service ready; hardware-mode defaults allocate no bus/worker/JSON heap");
    return err;
}

size_t amp_control_request(const uint8_t *p,size_t len,uint64_t session,uint8_t out[255])
{
    out[0]=MSG_AMP_CONTROL; out[1]=len>=2?p[1]|0x80:0xFF;
    out[2]=len>=3?p[2]:0; out[3]=BAD_LENGTH;
    if(len<3||p[0]!=MSG_AMP_CONTROL) return 4;
    if(!lock) { out[3]=NO_MEMORY; return 4; }
    const unsigned op=p[1]; const uint8_t token=p[2];
    take();
    if(upload && esp_timer_get_time()>upload_deadline) discard_upload();
    out[3]=OK;
    size_t size=4;
    switch(op) {
    case 0: /* STATUS. Actual IO result is separate from upload/queue ACK. */
        if(len!=3) { out[3]=BAD_LENGTH; break; }
        out[4]=1; put32(out+5,generation); put32(out+9,applied_generation);
        out[13]=applying||pending||run_pending;
        out[14]=last_token; out[15]=last_result;
        out[16]=hw_mute_is_playing(); out[17]=hw_mute_is_muted();
        for(unsigned i=0;i<AMP_PORTS;++i) {
            const size_t off=18+i*18;
            out[off]=status[i].state; out[off+1]=status[i].muted;
            put32(out+off+2,(uint32_t)status[i].error);
            put32(out+off+6,status[i].writes); put32(out+off+10,status[i].reads);
            put32(out+off+14,status[i].jobs);
        }
        out[54]=persisted;
        size=55; break;
    case 1: { /* GET JSON chunk. */
        if(len!=5) { out[3]=BAD_LENGTH; break; }
        const size_t off=u16(p+3);
        if(off>requested_len) { out[3]=BAD_VALUE; break; }
        size_t n=requested_len-off; if(n>243) n=243;
        out[4]=(uint8_t)requested_len; out[5]=(uint8_t)(requested_len>>8);
        out[6]=(uint8_t)off; out[7]=(uint8_t)(off>>8);
        put32(out+8,generation);
        memcpy(out+12,json_text()+off,n); size=12+n; break;
    }
    case 2: /* BEGIN: total bytes + expected generation. Never opens a bus. */
        if(len!=9) { out[3]=BAD_LENGTH; break; }
        if(u32(p+5)!=generation) { out[3]=STALE; break; }
        if(!u16(p+3)||u16(p+3)>AMP_JSON_MAX) { out[3]=BAD_VALUE; break; }
        /* An explicit new upload may replace a boot configuration whose task
         * could not start. Do not leave that recoverable OOM permanently busy. */
        if(pending&&!started&&!applying) { free(pending);pending=NULL; }
        if(applying||pending||run_pending) { out[3]=BUSY; break; }
        discard_upload(); upload=calloc(1,(size_t)u16(p+3)+1);
        if(!upload) { out[3]=NO_MEMORY; break; }
        upload_size=u16(p+3); upload_token=token; upload_session=session;
        upload_generation=generation; upload_deadline=esp_timer_get_time()+10000000; break;
    case 3: /* CHUNK: monotonically contiguous offsets, no hidden holes. */
        if(len<6) { out[3]=BAD_LENGTH; break; }
        if(!upload||upload_token!=token||upload_session!=session||upload_generation!=generation) { out[3]=STALE; break; }
        if(u16(p+3)!=upload_received||len-5>(size_t)(upload_size-upload_received)) { out[3]=BAD_VALUE; break; }
        memcpy(upload+upload_received,p+5,len-5); upload_received+=(uint16_t)(len-5);
        upload_deadline=esp_timer_get_time()+10000000; break;
    case 4: { /* COMMIT: validate first; optional independent amp NVS save. */
        if(len!=4||p[3]>1) { out[3]=BAD_LENGTH; break; }
        if(!upload||upload_token!=token||upload_session!=session||upload_generation!=generation||upload_received!=upload_size) { out[3]=STALE; break; }
        if(applying||pending||run_pending) { out[3]=BUSY; break; }
        amp_profile_t *candidate=calloc(1,sizeof(*candidate));
        char error[96];
        if(!candidate) { out[3]=NO_MEMORY; break; }
        if(!amp_profile_parse((char*)upload,upload_size,candidate,error,sizeof(error))) {
            free(candidate); out[3]=BAD_VALUE; break;
        }
        char *new_json=malloc((size_t)upload_size+1);
        if(!new_json) { free(candidate);out[3]=NO_MEMORY;break; }
        const bool worker=started||needs_bus(candidate);
        if(worker&&start_worker_locked()!=ESP_OK) {
            free(new_json);free(candidate);out[3]=NO_MEMORY;break;
        }
        if(p[3] && save_profile((char*)upload,upload_size)!=ESP_OK) {
            free(new_json);free(candidate); out[3]=SAVE_ERROR; break;
        }
        if(p[3]) persisted=true;
        memcpy(new_json,upload,upload_size); new_json[upload_size]=0;
        free(requested_json);requested_json=new_json;requested_len=upload_size;++generation;
        last_token=token;last_result=OK;
        if(worker) pending=candidate;
        else { free(candidate);applied_generation=generation; }
        discard_upload();
        put32(out+4,generation); size=8; break;
    }
    case 5: /* RUN a named user function by its stable profile index (>=3). */
        if(len!=5||p[3]>=AMP_PORTS||p[4]<3||p[4]>=AMP_FUNCTIONS_MAX) { out[3]=BAD_VALUE; break; }
        if(applying||pending||run_pending||status[p[3]].state!=READY) { out[3]=BUSY; break; }
        if(p[4]>=current_functions[p[3]]) { out[3]=BAD_VALUE; break; }
        run_port=p[3]; run_function=p[4]; run_token=token; run_pending=true; break;
    case 6: { /* Remove only the saved amp profile; current RAM is unchanged. */
        if(len!=3) { out[3]=BAD_LENGTH; break; }
        if(applying||pending||run_pending||upload) { out[3]=BUSY; break; }
        nvs_handle_t h;
        esp_err_t err=nvs_open("amp_i2c",NVS_READWRITE,&h);
        if(err==ESP_OK) {
            err=nvs_erase_key(h,"profile_v1");
            if(err==ESP_ERR_NVS_NOT_FOUND) err=ESP_OK;
            if(err==ESP_OK) err=nvs_commit(h);
            nvs_close(h);
        }
        if(err!=ESP_OK) out[3]=SAVE_ERROR; else persisted=false;
        break;
    }
    default: out[3]=BAD_VALUE; break;
    }
    give();
    return size;
}
