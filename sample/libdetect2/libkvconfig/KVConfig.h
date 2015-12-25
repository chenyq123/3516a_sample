/** 读取配置文件，格式为：
		# 注释行
		key1=value1
		key 2=value 2

 */

#pragma once

#include <vector>
#include <string>
#include <map>
#include <cc++/thread.h>

#ifdef WIN32
#	ifdef LIBKVCONFIG_EXPORTS
#		define KVCAPI __declspec(dllexport)
#	else
#		define KVCAPI __declspec(dllimport)
#	endif
#else
#	define KVCAPI
#endif //

class KVCAPI KVConfig
{
	typedef std::map<std::string, std::string> KVS;
	KVS kvs_;
	ost::Mutex cs_;
	std::string filename_;

public:
	KVConfig(const char *filename);
	~KVConfig(void);

	const char *file_name() const { return filename_.c_str(); }

	/// 返回是否包含 key
	bool has_key(const char *key);

	/// 返回 key 对应的 value，如果没有找到 key, 则返回 def
	const char *get_value(const char *key, const char *def = 0);

	// 删除 key
	bool del_key(const char *key);

	/// 返回 key 列表
	std::vector<std::string> keys();

	/// 支持新增或修改 key=value，如果 value == 0，则删除 key，如果 key 已经存在，则覆盖，
	int set_value(const char *key, const char *value);
	int set_value(const char *key, int v);

	/// 保存到指定的文件名
	int save_as(const char *filename);
	int reload();

	void clear();	// 删除所有

private:
	void load_from_file(const char *filename);
};
