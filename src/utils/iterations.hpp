//
// Created by mbero-akoko on 3/14/26.
//

#ifndef ROBOTDYNAMICS_ITERATIONS_HPP
#define ROBOTDYNAMICS_ITERATIONS_HPP
#include <bits/stdc++.h>

namespace utils::iterations {
    /**
 * @brief Finds the fixed point of a function f by applying it to its own
 * result, yielding each intermediate value.
 * * Equivalent to Python's:
 * x, f(x), f(f(x)), f(f(f(x)))...
 * * @tparam T The type of the state.
 * @tparam F A callable that takes T and returns T.
 * @param stepping_function The transformation function.
 * @param start_value The initial value.
 * @return std::generator<T> A C++23 coroutine-based generator.
 */
    template<typename T , std::invocable<T> F>
    requires std::convertible_to<std::invoke_result<F, T>, T>
    auto iterate(F stepping_function, T start_value) -> std::generator<T> {
        T state = std::move(start_value);
        while (true) {
            co_yield state;
            state = stepping_function(state);
        }
    };


    /**
    * @brief Return the last value of the given range.
    * * Returns std::nullopt if the range is empty.
    * If the range is infinite, this function will loop forever.
    * * @tparam R A range type.
    * @param values The range to iterate through.
    * @return std::optional of the value type of the range.
    */
    template<std::ranges::input_range R>
    auto last(R&& values) -> std::optional<std::ranges::range_value_t<R>> {
        auto iter = std::ranges::begin(values);
        auto sentinel = std::ranges::end(values);

        if (iter == sentinel) {
            return std::nullopt;
        }

        auto last_elem = *iter;
        while (++iter != sentinel) {
            last_elem = *iter;
        }
        return last_elem;
    }


    /**
     * @brief Read from a range until two consecutive values satisfy the
     * given done function or the input range ends.
     * * Throws std::runtime_error if the input range is empty.
     * * @tparam R An input range.
     * @tparam F A callable taking two elements and returning bool.
     */
    template<std::ranges::input_range R, typename F>
    requires std::invocable<F, std::ranges::range_value_t<R>, std::ranges::range_value_t<R>>
    auto converge(R&& values, F done) -> std::generator<std::ranges::range_value_t<R>> {
        auto iter = std::ranges::begin(values);
        auto sentinel = std::ranges::end(values);

        if (iter == sentinel) {
            throw std::runtime_error("converge: Input iterator is already empty. ");
        }

        using T = std::ranges::range_value_t<R>;
        T a = *iter;
        co_yield a;
        while (++ iter == sentinel) {
            T b = *iter;
            co_yield b;
            if (done(a, b)) {
                co_return;
            }
            a = std::move(b);
        }

    }



    namespace test_functionality {

        auto test_iterate() -> void {

            auto powers_of_two = iterate([](int x){return x**2; }, 1);
            std::cout << "First powers of two" << std::endl;
            std::vector<int> values = powers_of_two
                                      | std::views::take(10)
                                      | std::ranges::to<std::vector<int>>();
            std::copy(
                std::begin(values), std::end(values),
                std::ostream_iterator<int>(std::cout,"")
                );

        }

    }
}

#endif //ROBOTDYNAMICS_ITERATIONS_HPP