#include "blocking_ptr_ring.h"
#include <functional>
using namespace hesai::lidar;

template <typename T, size_t N>
T* BlockingPtrRing<T, N>::get_back_ptr() {
    LockC lock(_mutex);
    _condv.wait(lock, std::bind(&Super::not_empty, this));
    T* value = Super::get_back_ptr();
    _condv.notify_one();
    return value;
}

template <typename T, size_t N>
void BlockingPtrRing<T, N>::pop_back_ptr() {
    LockC lock(_mutex);
    _condv.wait(lock, std::bind(&Super::not_empty, this));
    Super::eff_pop_back();
    _condv.notify_one();
}

template <typename T, size_t N>
T* BlockingPtrRing<T, N>::get_back_next_ptr() {
    LockC lock(_mutex);
    _condv.wait(lock, std::bind(&Super::not_full, this));
    T* value = Super::get_back_next_ptr();
    _condv.notify_one();
    return value;
}

template <typename T, size_t N>
void BlockingPtrRing<T, N>::push_back_ptr() {
    LockC lock(_mutex);
    _condv.wait(lock, std::bind(&Super::not_full, this));
    Super::push_back_ptr();
    _condv.notify_one();
}

template <typename T, size_t N>
T* BlockingPtrRing<T, N>::get_front_ptr() {
    LockC lock(_mutex);
    _condv.wait(lock, std::bind(&Super::not_empty, this));
    T* value = Super::get_front_ptr();
    _condv.notify_one();
    return value;
}

template <typename T, size_t N>
void BlockingPtrRing<T, N>::pop_front_ptr() {
    LockC lock(_mutex);
    _condv.wait(lock, std::bind(&Super::not_empty, this));
    Super::eff_pop_front();
    _condv.notify_one();
}

template <typename T, size_t N>
T* BlockingPtrRing<T, N>::get_front_next_ptr() {
    LockC lock(_mutex);
    _condv.wait(lock, std::bind(&Super::not_full, this));
    T* value = Super::get_front_next_ptr();
    _condv.notify_one();
    return value;
}

template <typename T, size_t N>
void BlockingPtrRing<T, N>::push_front_ptr() {
    LockC lock(_mutex);
    _condv.wait(lock, std::bind(&Super::not_full, this));
    Super::push_front_ptr();
    _condv.notify_one();
}

template <typename T, size_t N>
bool BlockingPtrRing<T, N>::try_get_front_ptr(T** value) {
    LockC lock(_mutex);
    using namespace std::literals::chrono_literals;
    bool ret = _condv.wait_for(lock, 100ms, std::bind(&Super::not_empty, this));
    if (ret) {
        *value = Super::get_front_ptr();
    }
    else{
        ;
    }
    _condv.notify_one();
    return ret;
}

template <typename T, size_t N>
bool BlockingPtrRing<T, N>::empty()
{
    LockS lock(_mutex);
    return Super::empty();
}

template <typename T, size_t N>
bool BlockingPtrRing<T, N>::not_empty()
{
    LockS lock(_mutex);
    return Super::not_empty();
}

template <typename T, size_t N>
bool  BlockingPtrRing<T, N>::full()
{
    LockS lock(_mutex);
    return Super::full();
}

template <typename T, size_t N>
bool BlockingPtrRing<T, N>::not_full()
{
    LockS lock(_mutex);
    return Super::not_full();
}

template <typename T, size_t N>
void BlockingPtrRing<T, N>::clear()
{
    LockS lock(_mutex);
    Super::clear();
}

template <typename T, size_t N>
void BlockingPtrRing<T, N>::eff_clear()
{
    LockS lock(_mutex);
    Super::eff_clear();
}