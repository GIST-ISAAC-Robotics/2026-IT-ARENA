// 진단용: D3D12 core의 최초 로딩 시점을 유지하면서 조기 언매핑만 막습니다.
#include <cstring>
#include <dlfcn.h>

extern "C" void *dlopen(const char *filename, int flags) noexcept
{
  using Open = void *(*)(const char *, int);
  static auto real_open = reinterpret_cast<Open>(dlsym(RTLD_NEXT, "dlopen"));
  if (filename) {
    const char *base = std::strrchr(filename, '/');
    base = base ? base + 1 : filename;
    if (std::strcmp(base, "libd3d12core.so") == 0)
      flags |= RTLD_NODELETE;
  }
  return real_open(filename, flags);
}
