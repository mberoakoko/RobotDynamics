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