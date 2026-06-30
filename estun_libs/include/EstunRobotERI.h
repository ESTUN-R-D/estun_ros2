#ifndef EstunRobotERI_H__
#define EstunRobotERI_H__
namespace log4cpp
{
	class Category;
}
// 下列 ifdef 块是创建使从 DLL 导出更简单的
// 宏的标准方法。此 DLL 中的所有文件都是用命令行上定义的 ESTUNROBOTERI_EXPORTS
// 符号编译的。在使用此 DLL 的
// 任何其他项目上不应定义此符号。这样，源文件中包含此文件的任何其他项目都会将
// ESTUNROBOTERI_API 函数视为是从 DLL 导入的，而此 DLL 则将用此宏定义的
// 符号视为是被导出的。
#if defined(__GNUG__) && defined(__unix__)
	# if defined(ESTUNROBOTERI_EXPORTS)
	// Use INSTLIB_API for normal function exporting
	#  define ESTUNROBOTERI_API    __attribute__((visibility("default")))

	// Use INSTLIB_EXPORT for static template class member variables
	// They must always be 'globally' visible.
	#  define ESTUNROBOTERI_EXPORT __attribute__((visibility("default")))

	// Use INSTLIB_HIDE to explicitly hide a symbol
	#  define ESTUNROBOTERI_HIDE   __attribute__((visibility("hidden")))

	# else
	#  define ESTUNROBOTERI_API
	#  define ESTUNROBOTERI_EXPORT __attribute__((visibility("default")))
	#  define ESTUNROBOTERI_HIDE   __attribute__((visibility("hidden")))
	# endif
#else
	#ifdef ESTUNROBOTERI_EXPORTS
	#define ESTUNROBOTERI_API __declspec(dllexport)
	#else
	#define ESTUNROBOTERI_API __declspec(dllimport)
	#endif
#endif

// 此类是从 EstunRobotERI.dll 导出的
class ESTUNROBOTERI_API CEstunRobotERI {
public:
	CEstunRobotERI(void);
	// TODO: 在此添加您的方法。
};

extern ESTUNROBOTERI_API int nEstunRobotERI;
extern ESTUNROBOTERI_API log4cpp::Category *g_eriSDKLog;

ESTUNROBOTERI_API int fnEstunRobotERI(void);
#endif
