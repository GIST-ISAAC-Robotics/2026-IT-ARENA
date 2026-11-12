#pragma once
// ST3215의 확인된 RAM 명령/관측만. UART/실제 구동/기구 교정을 소유하지 않는다.
// 단일 소유자. pump/feed/tick은 비차단이며 호출자는 별도로 5ms 모터 주기를 유지한다.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace arena_servo {
enum class Phase { Idle, Sending, Waiting, Complete, Fault };
enum class Error { None, InvalidClock, Deadline, SinkContract, ServoStatus, Cancelled };
struct Response {
  std::int64_t started_us=0, completed_us=0;
  std::uint8_t id=0, error=0, size=0;
  std::array<std::uint8_t,2> data{};
};

class Transaction {
 public:
  // EEPROM/ID/baud/영점 쓰기 API는 없다. volatile goal도 장착 limits는 호출자가 검사한다.
  bool read_mode(std::int64_t now,unsigned id) { return read(now,id,33,1); }
  bool read_position(std::int64_t now,unsigned id) { return read(now,id,56,2); }
  bool torque(std::int64_t now,unsigned id,bool enabled) {
    const std::uint8_t p[]={40,static_cast<std::uint8_t>(enabled)};
    return begin(now,id,3,p,2,0);
  }
  bool goal(std::int64_t now,unsigned id,unsigned position,unsigned speed,unsigned acceleration) {
    if(position>4095 || speed==0 || speed>400 || acceleration<5 || acceleration>20)return false;
    const std::uint8_t p[]={41,static_cast<std::uint8_t>(acceleration),
      static_cast<std::uint8_t>(position),static_cast<std::uint8_t>(position>>8),0,0,
      static_cast<std::uint8_t>(speed),static_cast<std::uint8_t>(speed>>8)};
    return begin(now,id,3,p,8,0);
  }
  template<class Sink>
  void pump(std::int64_t now,Sink&& sink) {
    tick(now);
    if(phase_!=Phase::Sending)return;
    const auto remaining=tx_size_-tx_offset_;
    const auto accepted=sink(tx_.data()+tx_offset_,remaining);
    if(accepted>remaining){fail(Error::SinkContract);return;}
    tx_offset_+=accepted;
    if(tx_offset_==tx_size_)phase_=Phase::Waiting;
  }
  void feed(std::int64_t now,const std::uint8_t* bytes,std::size_t size) {
    tick(now);
    if(phase_!=Phase::Waiting || bytes==nullptr)return;
    // HAL은 최대64B씩 넘긴다. 남은 입력은 다음 주기에 처리해 제어 주기를 막지 않는다.
    if(size>64){fail(Error::SinkContract);return;}
    for(std::size_t i=0;i<size && phase_==Phase::Waiting;++i){
      if(rx_size_==rx_.size())drop(1);
      rx_[rx_size_++]=bytes[i];parse(now);
    }
  }
  void tick(std::int64_t now) {
    if(now<0 || (has_clock_ && now<last_clock_)){fail(Error::InvalidClock);return;}
    has_clock_=true;last_clock_=now;
    if((phase_==Phase::Sending || phase_==Phase::Waiting) && now>=deadline_)
      fail(Error::Deadline);
  }
  bool take(Response& out) {
    if(phase_!=Phase::Complete)return false;
    out=response_;phase_=Phase::Idle;return true;
  }
  void cancel() { fail(Error::Cancelled); }
  // 오류/STOP 뒤 UART TX/RX를 외부에서 비운 새 연결 경계에서만 호출한다.
  // ST 응답에 요청 ID가 없어 늦은 동일 길이 응답을 이 객체만으로 식별할 수 없다.
  bool reset_after_drain(std::int64_t now) {
    if(now<0 || (has_clock_ && now<last_clock_))return false;
    phase_=Phase::Idle;error_=Error::None;rx_size_=tx_size_=tx_offset_=0;
    last_clock_=now;has_clock_=true;response_=Response{};return true;
  }
  Phase phase() const { return phase_; }
  Error error() const { return error_; }
  std::int64_t deadline() const { return deadline_; }
  static constexpr std::int64_t timeout_us=30000; // 기존 실측 probe의 응답 한도, 교정값 아님
 private:
  bool read(std::int64_t now,unsigned id,std::uint8_t address,std::uint8_t size) {
    const std::uint8_t p[]={address,size};return begin(now,id,2,p,2,size);
  }
  bool begin(std::int64_t now,unsigned id,std::uint8_t instruction,
             const std::uint8_t* params,std::size_t count,std::uint8_t expected) {
    tick(now);
    if(phase_!=Phase::Idle || id>253 || now>std::numeric_limits<std::int64_t>::max()-timeout_us)return false;
    tx_[0]=255;tx_[1]=255;tx_[2]=id;tx_[3]=count+2;tx_[4]=instruction;
    std::memcpy(tx_.data()+5,params,count);
    unsigned sum=0;for(std::size_t i=2;i<count+5;++i)sum+=tx_[i];
    tx_[count+5]=static_cast<std::uint8_t>(~sum);
    tx_size_=count+6;tx_offset_=rx_size_=0;expected_=expected;
    deadline_=now+timeout_us;response_=Response{};response_.started_us=now;response_.id=id;
    phase_=Phase::Sending;error_=Error::None;return true;
  }
  void fail(Error e) { phase_=Phase::Fault;error_=e; }
  void drop(std::size_t count) {
    count=std::min(count,rx_size_);rx_size_-=count;
    std::memmove(rx_.data(),rx_.data()+count,rx_size_);
  }
  void parse(std::int64_t now) {
    for(;;){
      while(rx_size_>=2 && (rx_[0]!=255 || rx_[1]!=255))drop(1);
      if(rx_size_<4)return;
      // 관측2B/상태0B뿐 아니라 로컬 송신 echo 최대14B를 안전하게 소비한다.
      if(rx_[2]>253 || rx_[3]<2 || rx_[3]>10){drop(1);continue;}
      const auto size=static_cast<std::size_t>(rx_[3])+4;
      if(rx_size_<size)return;
      unsigned sum=0;for(std::size_t i=2;i<size;++i)sum+=rx_[i];
      if((sum&255)!=255){drop(1);continue;}
      if((size==tx_size_ && std::memcmp(rx_.data(),tx_.data(),size)==0) ||
         rx_[2]!=response_.id || rx_[3]!=expected_+2){drop(size);continue;}
      response_.completed_us=now;response_.error=rx_[4];response_.size=expected_;
      if(expected_)std::memcpy(response_.data.data(),rx_.data()+5,expected_);
      if(response_.error){fail(Error::ServoStatus);return;}
      phase_=Phase::Complete;return;
    }
  }
  Phase phase_=Phase::Idle;
  Error error_=Error::None;
  bool has_clock_=false;
  std::int64_t last_clock_=0,deadline_=0;
  std::array<std::uint8_t,14> tx_{};
  std::array<std::uint8_t,18> rx_{};
  std::size_t tx_size_=0,tx_offset_=0,rx_size_=0;
  std::uint8_t expected_=0;
  Response response_{};
};
} // namespace arena_servo
