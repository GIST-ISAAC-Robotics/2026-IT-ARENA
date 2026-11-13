// WSL Mesa/D3D12의 렌더 스레드 종료 전 core 언매핑을 막는 한정적 우회입니다.
// 드라이버를 미리 초기화하지 않고 libd3d12core.so의 실제 dlopen에만 적용합니다.
// 근거와 제거 조건: docs/simulation/GPU_RENDERING.md
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
