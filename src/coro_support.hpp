#pragma once

#include <coroutine>
#include <exception>
#include <concepts>



struct Void {};

//taken from https://en.cppreference.com/w/cpp/language/coroutines
// (originally named Generator)
//but then adapted and simplified for my usecase
struct SideEffectCoroutine
{
    struct promise_type;
    using handle_type = std::coroutine_handle<promise_type>;

    struct promise_type // required
    {
        std::exception_ptr exception;

        SideEffectCoroutine get_return_object()
        {
            return SideEffectCoroutine(handle_type::from_promise(*this));
        }
        std::suspend_always initial_suspend() { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }
        void unhandled_exception() { this->exception = std::current_exception(); } // saving exception

        std::suspend_always yield_value(Void) { return {}; }
        void return_void() { }
    };

    handle_type handle;

    SideEffectCoroutine(handle_type h) : handle(h) {}
    ~SideEffectCoroutine() { this->handle.destroy(); }

    explicit operator bool() { return !this->handle.done(); }

    void operator()()
    {
        this->handle();
        if (this->handle.promise().exception) {
            // propagate coroutine exception in called context
            std::rethrow_exception(handle.promise().exception);
        }
    }
}; //class SideEffectCoroutine

#define WAIT_WHILE(x) while (x) co_yield Void{}
#define YIELD co_yield Void{}

//assumes init is an expression returning SideEffectCoroutine.
//executes one step of that coroutine until it has finished or cond is no longer true.
//(thus assumes usage inside a coroutine itself)
#define EXEC_WHILE(cond, init) {\
    SideEffectCoroutine f = init;\
    while ((cond) && f) {\
        f();\
        co_yield Void{};\
    }\
}

#define EXEC(init) EXEC_WHILE(true, init)
