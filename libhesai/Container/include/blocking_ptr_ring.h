#ifndef ___HESAI__CONTAINER__BLOCKING_PTR_RING_HH___
#define ___HESAI__CONTAINER__BLOCKING_PTR_RING_HH___

#include "ring.h"
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <chrono>
#include <functional>
namespace hesai
{
namespace lidar
{
template <typename T, size_t N>
class BlockingPtrRing : public Ring<T, N>{
public:
    using Super = Ring<T, N>;
    using Mutex = std::mutex;
    using Condv = std::condition_variable;
    using LockS = std::lock_guard<Mutex>;           // lock in scope
    using LockC = std::unique_lock<Mutex>;          // lock by condition
private:
    Mutex _mutex;
    Condv _condv;
public:
    T* get_back_ptr();
    void pop_back_ptr();
    T* get_back_next_ptr();
    void push_back_ptr();
    T* get_front_ptr();
    void push_front_ptr();
    T* get_front_next_ptr();
    void pop_front_ptr();
    bool try_get_front_ptr(T**);
    bool empty();
    bool not_empty();
    bool full();
    bool not_full();
    void clear();
    void eff_clear();
};
}  // namespace lidar
}  // namespace hesai

#include "blocking_ptr_ring.cc"

#endif // !___HESAI__CONTAINER__BLOCKING_PTR_RING_HH___
