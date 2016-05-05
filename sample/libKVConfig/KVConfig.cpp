#include <stdio.h>
#include <stdlib.h>
#include "KVConfig.h"

KVConfig::KVConfig(const char *filename)
{
    this->filename_ = filename;

    reload();
}

KVConfig::~KVConfig(void)
{
}

bool KVConfig::has_key(const char *key)
{
    //ost::MutexLock al(cs_);

    return get_value(key) != 0;
}

bool KVConfig::del_key(const char *key)
{
    //ost::MutexLock al(cs_);

    KVS::iterator itf = kvs_.find(key);
    if (itf != kvs_.end()) {
        kvs_.erase(itf);
        return true;
    }
    else
        return false;
}

const char *KVConfig::get_value(const char *key, const char *def)
{
    //ost::MutexLock al(cs_);

    KVS::const_iterator itf = kvs_.find(key);
    if (itf != kvs_.end())
        return itf->second.c_str();
    return def;
}

std::vector<std::string> KVConfig::keys()
{
    //ost::MutexLock al(cs_);

    std::vector<std::string> ks;
    KVS::const_iterator it;
    for (it = kvs_.begin(); it != kvs_.end(); ++it)
        ks.push_back(it->first);

    return ks;
}

int KVConfig::set_value(const char *key, const char *value)
{
    //ost::MutexLock al(cs_);

    KVS::iterator itf = kvs_.find(key);
    if (itf == kvs_.end()) {
        if (value) {
            kvs_[key] = value;
        }
    }
    else {
        if (value) {
            itf->second = value;
        }
        else {
            kvs_.erase(itf);
        }
    }

    return 0;
}

int KVConfig::set_value(const char *key, int v)
{
    char info[64];
    snprintf(info, sizeof(info), "%d", v);
    return set_value(key, info);
}

int KVConfig::save_as(const char *filename)
{
    //ost::MutexLock al(cs_);
    //std::string tmp;

    //if (!filename) {
    //    tmp = filename_ + ".session";
    //    filename = tmp.c_str();
    //}
    if(filename == 0)
    {
        filename = this->filename_.c_str();
    }
    FILE *fp = fopen(filename, "w");
    if (!fp) return -1;

    KVS::const_iterator it;
    for (it = kvs_.begin(); it != kvs_.end(); ++it) {
        fprintf(fp, "%s=%s\n", it->first.c_str(), it->second.c_str());
    }
    fclose(fp);

    return 0;
}

int KVConfig::reload()
{
    kvs_.clear();

    load_from_file(filename_.c_str());

//  std::string tmp = filename_ + ".session";
//  load_from_file(tmp.c_str());
    return 0;
}

void KVConfig::clear()
{
    //ost::MutexLock al(cs_);
    kvs_.clear();
}

void KVConfig::load_from_file(const char *filename)
{
    //ost::MutexLock al(cs_);

    FILE *fp = fopen(filename, "r");
    if (!fp) {
        fprintf(stderr, "ERR: %s: can't open file '%s'\n", __FUNCTION__, filename);
        return;
    }

    while (!feof(fp)) {
        char line[1024];    // FIXME: 这里简单的假设一行不会超过 1024 字节
        char *p = fgets(line, sizeof(line), fp);
        if (!p) continue;
        while (p && isspace(*p)) p++;   // 去除行首空格
        if (*p == '#') continue;        // 注释行

        char key[64], value[512];   // FIXME：
        if (sscanf(p, "%63[^=] = %511[^\r\n]", key, value) == 2) {
            kvs_[key] = value;
            //printf("key:%s,value:%s\n",key,value);
        }
    }

    fclose(fp);
}
