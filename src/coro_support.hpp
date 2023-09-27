#pragma once

#include <coroutine>
#include <exception>
#include <concepts>
#include <array>

struct GlobalOwner {}; //signals usual heap allocation of the coroutine state

//coroutines can call each other. one such call chain behaves exactly like the usual call stack
// , as long as no coroutine manages multiple coroutines simultaniously itself.
//for that special case, one can circumvent heap allocation and give each such call chain its own stack.
template<typename T>
concept CallstackOwner = std::is_same_v<T, GlobalOwner> || requires {
    { T::coroutines_stack_size } -> std::convertible_to<std::size_t>;
    { T::name } -> std::convertible_to<char const*>;
};

//there is only one callstack per type, as coroutines are unable to get an allocator object passed in the constructor.
// thus everything here is static.
template<CallstackOwner O>
class CoroutineStack {
    constinit static std::size_t start_unused;
    //std::size_t has pointer allignment -> every element of arena has pointer allignment 
    // and can thus be a valid starting position for requested space 
    // (as long as no artificial allignment was specified for the type allocated...)
    constinit static std::array<std::size_t, O::coroutines_stack_size> arena;
    static constexpr auto elem_size = sizeof(std::size_t);

    static void* next_address() {
        return &arena[start_unused];
    }

public:
    static void* allocate(std::size_t n) {
        //n is given in bytes -> choose smallest multiple of std::size_t large enough to fit n bytes
        auto const nr_needed = (n + elem_size - 1) / elem_size;
        auto const new_start_unused = start_unused + nr_needed;
        assert(new_start_unused <= arena.size());
        void* const result = next_address();
        start_unused = new_start_unused;
        std::cout << "ALLOC " << nr_needed << " for " << O::name << "\n";
        return result;
    }

    static void deallocate(void* address) {
        assert(address < next_address());
        std::size_t const as_arena_index = (std::size_t*)address - arena.data();
        std::cout << "FREE " << (start_unused - as_arena_index) << " for " << O::name << "\n";
        assert(as_arena_index < arena.size());
        start_unused = as_arena_index;
    }
};

template<CallstackOwner O>
constinit std::size_t CoroutineStack<O>::start_unused = 0; //call stack is initially empty

//kinda doenst matter, how the stack is initialized. there is nothing stored here at the program start.
template<CallstackOwner O>
constinit std::array<std::size_t, O::coroutines_stack_size> CoroutineStack<O>::arena = {};



//return type for coroutine type below
//not returning any information is important, as this allows us call the coroutine exactly when
//the call operator is used and no call when the bool operator is evaluated is nessecairy.
//This is important when reasoning about the order of side effects caused by multiple coroutines occuring
struct Void {};

//taken from https://en.cppreference.com/w/cpp/language/coroutines
// (originally named Generator)
//but then adapted and simplified for this usecase
template<CallstackOwner O = GlobalOwner>
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

        void* operator new(std::size_t n) requires (!std::is_same_v<O, GlobalOwner>)
        {
            return CoroutineStack<O>::allocate(n);
        }

        void operator delete(void* address) requires (!std::is_same_v<O, GlobalOwner>) {
            CoroutineStack<O>::deallocate(address);
        }
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

//simplify implementation of coroutine type above by hiding the trivial return value
//these are macros, because the whole point of coroutines is to not behave like funktions.
#define WAIT_WHILE(x) while (x) co_yield Void{}
#define YIELD co_yield Void{}

//assumes init is an expression returning SideEffectCoroutine.
//executes one step of that coroutine until it has finished or cond is no longer true.
//(thus assumes usage inside a coroutine itself)
#define EXEC_WHILE(cond, init) {\
    SideEffectCoroutine coro_f = init;\
    while ((cond) && coro_f) {\
        coro_f();\
        co_yield Void{};\
    }\
}

#define EXEC(init) EXEC_WHILE(true, init)
