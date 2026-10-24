#include "amp_profile.h"
#include "cJSON.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

/* The allowed set excludes both I2S ports, damaged GPIO6, mute GPIO8, flash,
 * USB, console/JTAG and strapping pins. GPIO9/10 are the board defaults.
 * An unsupported pin is refused rather than silently replacing another owner. */
bool amp_profile_pin_safe(int pin)
{
    return pin == 5 || (pin >= 9 && pin <= 12) ||
           (pin >= 17 && pin <= 25) || (pin >= 35 && pin <= 47);
}

/* TI TAS5825M 9.6.1.3 / TAS5828M 8.1.3: CTRL_STATE[1:0], MUTE[3],
 * DIS_DSP[4]. Do not change reserved bit2 or confuse bit4 with a channel mute.
 * Clocks must be running before releasing DSP reset. Board-specific PPC3
 * coefficients belong in an explicit user profile, not invented defaults. */
static const char defaults[] =
"{\"version\":1,\"bus\":{\"enabled\":false,\"sda\":9,\"scl\":10,\"hz\":100000},\"ports\":["
"{\"port\":0,\"enabled\":false,\"model\":\"TAS5828M\",\"address\":76,\"functions\":{"
"\"startup\":[{\"write\":[0,0]},{\"write\":[127,0]},{\"update\":[3,27,10]},{\"delay_ms\":5},{\"update\":[3,3,3]}],"
"\"mute\":[{\"write\":[0,0]},{\"write\":[127,0]},{\"update\":[3,8,8]}],"
"\"unmute\":[{\"write\":[0,0]},{\"write\":[127,0]},{\"update\":[3,8,0]}]}},"
"{\"port\":1,\"enabled\":false,\"model\":\"TAS5825M\",\"address\":77,\"functions\":{"
"\"startup\":[{\"write\":[0,0]},{\"write\":[127,0]},{\"update\":[3,27,10]},{\"delay_ms\":5},{\"update\":[3,3,3]}],"
"\"mute\":[{\"write\":[0,0]},{\"write\":[127,0]},{\"update\":[3,8,8]}],"
"\"unmute\":[{\"write\":[0,0]},{\"write\":[127,0]},{\"update\":[3,8,0]}]}}]}";
const char *amp_profile_default_json(void) { return defaults; }

static bool fail(char *e, size_t n, const char *message)
{
    if (n) snprintf(e, n, "%s", message);
    return false;
}
static bool keys(const cJSON *o, const char *const *allowed, size_t count)
{
    if (!cJSON_IsObject(o)) return false;
    for (const cJSON *p=o->child; p; p=p->next) {
        bool found=false;
        for (size_t i=0; i<count; ++i) if (!strcmp(p->string,allowed[i])) found=true;
        if (!found) return false;
        for (const cJSON *q=p->next; q; q=q->next)
            if (!strcmp(p->string,q->string)) return false;
    }
    return true;
}
static bool integer(const cJSON *v, unsigned lo, unsigned hi, unsigned *out)
{
    if (!cJSON_IsNumber(v) || !isfinite(v->valuedouble) ||
        v->valuedouble < lo || v->valuedouble > hi || floor(v->valuedouble)!=v->valuedouble) return false;
    *out=(unsigned)v->valuedouble; return true;
}
static bool field(const cJSON *o, const char *k, unsigned lo, unsigned hi, unsigned *out)
{ return integer(cJSON_GetObjectItemCaseSensitive(o,k),lo,hi,out); }
static bool flag(const cJSON *o, const char *k, bool *out)
{
    const cJSON *v=cJSON_GetObjectItemCaseSensitive(o,k);
    if (!cJSON_IsBool(v)) return false;
    *out=cJSON_IsTrue(v); return true;
}
static bool name_ok(const char *s)
{
    if (!s || !s[0] || strlen(s)>=AMP_NAME_MAX) return false;
    for (;*s;++s) if (!((*s>='A'&&*s<='Z')||(*s>='a'&&*s<='z')||
                     (*s>='0'&&*s<='9')||*s=='_'||*s=='-')) return false;
    return true;
}
static bool sequence(const cJSON *a, amp_port_profile_t *port, amp_function_t *f)
{
    if (!cJSON_IsArray(a) || !a->child) return false;
    f->first=port->op_count;
    unsigned delay=0;
    for (const cJSON *v=a->child; v; v=v->next) {
        if (!cJSON_IsObject(v) || !v->child || v->child->next || port->op_count>=AMP_OPS_MAX) return false;
        amp_op_t *op=&port->ops[port->op_count++];
        const cJSON *x=v->child;
        if (!strcmp(x->string,"delay_ms")) {
            unsigned ms;
            if (!integer(x,1,1000,&ms)) return false;
            delay+=ms;
            if (delay > (!strcmp(f->name,"startup") ? 2000u : 100u)) return false;
            op->kind=AMP_DELAY; op->delay_ms=(uint16_t)ms;
            continue;
        }
        if (!cJSON_IsArray(x)) return false;
        const int n=cJSON_GetArraySize(x);
        bool write=!strcmp(x->string,"write"), update=!strcmp(x->string,"update"), check=!strcmp(x->string,"check");
        if ((!write&&!update&&!check) || (write ? n<2||n>AMP_BURST_MAX+1 : n!=3)) return false;
        unsigned values[AMP_BURST_MAX+1];
        for (int i=0;i<n;++i) if (!integer(cJSON_GetArrayItem(x,i),0,255,&values[i])) return false;
        op->reg=(uint8_t)values[0];
        if (write) {
            if (values[0]+(unsigned)n-2>255) return false;
            op->kind=AMP_WRITE; op->count=(uint8_t)(n-1);
            for (int i=1;i<n;++i) op->data[i-1]=(uint8_t)values[i];
        } else {
            if (!values[1] || (values[2] & ~values[1])) return false;
            op->kind=update?AMP_UPDATE:AMP_CHECK; op->mask=(uint8_t)values[1];
            op->data[0]=(uint8_t)values[2]; op->count=1;
        }
    }
    f->count=port->op_count-f->first;
    return true;
}

bool amp_profile_parse(const char *json, size_t len, amp_profile_t *out, char *e, size_t n)
{
    if (!json||!out||!len||len>AMP_JSON_MAX||memchr(json,0,len)) return fail(e,n,"JSON length");
    /* Bound parser recursion before cJSON sees untrusted input. Quotes and
     * backslash escapes are tracked, so braces in a string do not affect it. */
    unsigned depth=0; bool quote=false,escape=false;
    for (size_t i=0;i<len;++i) {
        const char c=json[i];
        if (c=='\\' && i+5<len && !memcmp(json+i+1,"u0000",5))
            return fail(e,n,"NUL escape");
        if (quote) { if(escape) escape=false; else if(c=='\\') escape=true; else if(c=='"') quote=false; }
        else if(c=='"') quote=true;
        else if(c=='{'||c=='[') { if(++depth>12) return fail(e,n,"JSON depth"); }
        else if(c=='}'||c==']') { if(!depth--) return fail(e,n,"JSON brackets"); }
    }
    if (depth||quote) return fail(e,n,"JSON incomplete");
    const char *end=NULL;
    cJSON *root=cJSON_ParseWithLengthOpts(json,len,&end,false);
    if (!root) return fail(e,n,"JSON syntax");
    while (end<json+len && (*end==' '||*end=='\t'||*end=='\r'||*end=='\n')) ++end;
    bool ok=false;
    static const char *const rkeys[]={"version","bus","ports"};
    static const char *const bkeys[]={"enabled","sda","scl","hz"};
    static const char *const pkeys[]={"port","enabled","model","address","functions"};
    unsigned v,sda,scl,hz;
    const cJSON *bus=cJSON_GetObjectItemCaseSensitive(root,"bus"), *ports=cJSON_GetObjectItemCaseSensitive(root,"ports");
    memset(out,0,sizeof(*out));
    if(end!=json+len || !keys(root,rkeys,3)||!field(root,"version",1,1,&v)||
       !keys(bus,bkeys,4)||!flag(bus,"enabled",&out->enabled)||
       !field(bus,"sda",0,63,&sda)||!field(bus,"scl",0,63,&scl)||
       !amp_profile_pin_safe((int)sda)||!amp_profile_pin_safe((int)scl)||sda==scl||
       !field(bus,"hz",100000,400000,&hz)||(hz!=100000&&hz!=400000)||
       !cJSON_IsArray(ports)||cJSON_GetArraySize(ports)!=AMP_PORTS) goto done;
    out->sda=(uint8_t)sda; out->scl=(uint8_t)scl; out->hz=hz;
    unsigned seen=0;
    for(const cJSON *p=ports->child;p;p=p->next) {
        unsigned index,address;
        if(!keys(p,pkeys,5)||!field(p,"port",0,AMP_PORTS-1,&index)||
           (seen&(1u<<index))||!field(p,"address",8,119,&address)) goto done;
        seen|=1u<<index;
        amp_port_profile_t *dst=&out->ports[index];
        const cJSON *model=cJSON_GetObjectItemCaseSensitive(p,"model"), *functions=cJSON_GetObjectItemCaseSensitive(p,"functions");
        if(!flag(p,"enabled",&dst->enabled)||!cJSON_IsString(model)||!name_ok(model->valuestring)||!cJSON_IsObject(functions)) goto done;
        strcpy(dst->model,model->valuestring); dst->address=(uint8_t)address;
        const char *reserved[]={"startup","mute","unmute"};
        for(unsigned i=0;i<3;++i) {
            amp_function_t *f=&dst->functions[dst->function_count++];
            strcpy(f->name,reserved[i]);
            if(!sequence(cJSON_GetObjectItemCaseSensitive(functions,f->name),dst,f)) goto done;
        }
        for(const cJSON *f=functions->child;f;f=f->next) {
            if(!name_ok(f->string)) goto done;
            for(const cJSON *q=f->next;q;q=q->next) if(!strcmp(f->string,q->string)) goto done;
            if(!strcmp(f->string,"startup")||!strcmp(f->string,"mute")||!strcmp(f->string,"unmute")) continue;
            if(dst->function_count>=AMP_FUNCTIONS_MAX) goto done;
            amp_function_t *entry=&dst->functions[dst->function_count++]; strcpy(entry->name,f->string);
            if(!sequence(f,dst,entry)) goto done;
        }
    }
    if(out->ports[0].enabled&&out->ports[1].enabled&&out->ports[0].address==out->ports[1].address) goto done;
    ok=true;
done:
    cJSON_Delete(root);
    return ok ? true : fail(e,n,"Invalid amplifier schema, pin, address or operation");
}

int amp_profile_op(const amp_op_t *op, const amp_bus_io_t *io)
{
    if(!op||!io||!io->write||!io->read) return -1;
    if(op->kind==AMP_DELAY) return 0;
    if(op->kind==AMP_WRITE) return io->write(io->context,op->reg,op->data,op->count);
    uint8_t value=0;
    int err=io->read(io->context,op->reg,&value,1);
    if(err) return err;
    if(op->kind==AMP_CHECK) return (value&op->mask)==op->data[0] ? 0 : -2;
    if(op->kind!=AMP_UPDATE) return -1;
    value=(value&~op->mask)|op->data[0];
    return io->write(io->context,op->reg,&value,1);
}
