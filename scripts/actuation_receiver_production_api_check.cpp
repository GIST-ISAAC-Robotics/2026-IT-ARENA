// 시험 매크로 없는 일반 빌드에서는 카운터 주입 API가 노출되지 않아야 한다.
#include "actuation_receiver.hpp"
#include <type_traits>
#include <utility>

template<class T, class = void> struct HasCounterSetter : std::false_type {};
template<class T> struct HasCounterSetter<T, std::void_t<decltype(std::declval<T&>().test_set_counters(0, 0))>>
    : std::true_type {};
static_assert(!HasCounterSetter<arena_actuation::ActuationReceiverCore>::value,
              "test-only state injection must not be exposed by the normal core build");

int main() {
  const std::uint32_t boot[] = {'b'};
  arena_actuation::ActuationReceiverCore r(arena_actuation::ReceiverConfig{}, boot, 1);
  return r.state() == arena_actuation::ReceiverState::kDisarmed && r.target_speed() == 0. ? 0 : 1;
}
