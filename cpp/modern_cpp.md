Modern C++ Note
===

> The first time I met modern C++ is in Bitbot project, written by [God Yuan](https://lmy.name). I found it is extremely efficient and concise, while it is hard to understand. Then I decided to use 
> 
> Dknt 2025.1

# C++11

## Curiously Recurring Template Pattern (CRTP)

CRTP is to achieve compile-time polymorphism. Sample code:

```cpp
template <class Derived>
struct base {
  void interface() {
    static_cast<Derived*>(this)->implementation();
  }
};
struct derived : base<derived> {
  void implementation() {
      // ...
  }
};
```

## Template Metaprogramming (C++11)

Compile-time class generation. Sample code:

```cpp
template <unsigned N>
struct factorial {
	static constexpr unsigned value = N * factorial<N - 1>::value;
};
template <>
struct factorial<0> {
	static constexpr unsigned value = 1;
};
factorial<5>::value; // 120
factorial<0>::value; // 1
```

> `constexpr` is a C++11 feature.

Another example:

```cpp
template <unsigned N>
struct fibonacci {
	static constexpr unsigned value = fibonacci<N - 1>::value + fibonacci<N - 2>::value;
};
template <>
struct fibonacci<0> {
	static constexpr unsigned value = 0;
};
template <>
struct fibonacci<1> {
	static constexpr unsigned value = 1;
};
fibonacci<10>::value; // 55
```

Compile-time code optimization.

```cpp
template <int length>
Vector<length>& Vector<length>::operator+=(const Vector<length>& rhs) 
{
    for (int i = 0; i < length; ++i)
        value[i] += rhs.value[i];
    return *this;
}
// The following code may be produced at runtime:
template <>
Vector<2>& Vector<2>::operator+=(const Vector<2>& rhs) 
{
    value[0] += rhs.value[0];
    value[1] += rhs.value[1];
    return *this;
}
```

Static table generation (see `constexpr` lambda for the simpler implementation):

```cpp
constexpr int TABLE_SIZE = 10;
template <int INDEX = 0, int... D>
struct Helper : Helper<INDEX + 1, D..., INDEX * INDEX> {};
template <int... D>
struct Helper<TABLE_SIZE, D...> {
  static constexpr std::array<int, TABLE_SIZE> table = {D...};
};
constexpr std::array<int, TABLE_SIZE> table = Helper<>::table;
// static constexpr std::array<int, 10UL> Helper<10, 0, 1, 4, 9, 16, 25, 36, 49, 64, 81>::table = {0, 1, 4, 9, 16, 25, 36, 49, 64, 81}
```

## `constexpr` (C++11)

`constexpr` is a C++11 feature, which allows you to declare variables and functions that can be evaluated at compile time.

`constexpr` variables must be initialized with a constant expression. `constexpr` functions can also be evaluated at compile time if all their arguments are constant expressions.

In C++11, `constexpr` has some restrictions:

* Could only contain a single return statement
* Couldn't contain loops, if statements, or local variables
* Had to return a value in a single expression

`constexpr` is further improved in C++14, since then it is widely used.

Example:

```cpp
constexpr int add(int a, int b) {
  return a + b;  // Allowed
}
constexpr int add_multi(int a, int b) {
  a += b;
  return a + b;  // Not allowed in C++11
}
```

## Variadic Template (C++11)

This feature allows you to write functions that can take a variable number of arguments. It is usually used in a recursive way.

Here is a sample code for `printf` in C, just feel it:

```cpp
void my_printf(const char *s) {
  while (*s) {
    if (*s == '%') {
      if (*(s + 1) == '%')
        ++s;
      else
        throw std::runtime_error("invalid format string: missing arguments");
      std::cout << *s++;
    }
  }
}
template <typename T, typename... Args>
void my_printf(const char *s, T value, Args... args) {
  while (*s) {
    if (*s == '%') {
      if (*(s + 1) != '%') {
        std::cout << value;
        s += 2;
        my_printf(s, args...);
        return;
      }
      ++s;
    }
    std::cout << *s++;
  }
}
// Test
my_printf("Hello, %d, %s\n", 1, "world!");
```

There is no simple mechanism to iterate over a variadic template, but you can use `std::tuple` to do that.

The operator `sizeof...` is used to get the number of arguments in a variadic template.

```cpp
template <typename... Args>
void print_args(Args... args) {
  std::cout << sizeof...(args) << std::endl;
}
print_args(1, 2, 3);  // Output: 3
```

Another example:

```cpp
template <size_t N>
struct CompileTimeString {
  constexpr CompileTimeString(const char (&str)[N]) {
    std::copy_n(str, N, name);
  }
  char name[N]{};
};
template<CompileTimeString... args>
std::array<std::string, sizeof...(args)> func3() {
  std::array<std::string, sizeof...(args)> arr = {args.name...};
  return arr;
}
auto res = func3<"Hello", "world!">();
// static constexpr std::array<std::string, 2UL> res = {{"Hello", "world!"}}
```

## Universal Reference (C++11)

Universal Reference (a.k.a. Perfect Forwarding) is a C++11 feature that allows you to bind a reference to any type of argument. Its behavior depends on the context. If an **Lvalue** is passed, the reference is `T&`. If an **Rvalue** is passed, the reference is `T&&`.

```cpp
template <typename T>
void foo(T&& x) {
  // x is a universal reference
}
foo(std::string("hello"));  // T is std::string&& (rvalue reference)
std::string s = "world";
foo(s);  // T is std::string& (lvalue reference)
```

## `std::tuple` (C++11)

`std::tuple` is a C++11 feature that allows you to create a fixed-size collection of heterogeneous objects.

Example:

```cpp
std::tuple<int, std::string, double> t(1, "hello", 3.14);
std::tuple<int, std::string, double> t2 = std::make_tuple(1, "hello", 3.14);
std::cout << std::get<0>(t) << std::endl;  // Output: 1
```

## Initializer List (C++11)

Initializer List is a C++11 feature that allows you to initialize a container with a list of values.

Example:

```cpp
std::vector<int> v = {1, 2, 3, 4, 5};  // Initialize a vector with a list of values

// Construct a initializer list
std::initializer_list<int> il = {1, 2, 3, 4, 5};
std::vector<int> v2(il);  // Initialize a vector with a initializer list
```

# C++14

## `constexpr` Enhancements (C++14)

> `constexpr` is introduced in C++11, but it is improved in C++14.

C++14 made `constexpr` much more flexible:

* Allowed multiple statements
* Allowed loops and if statements
* Allowed local variables
* Allowed mutation of objects declared within the function
* Relaxed many restrictions on constexpr member functions

Example:

```cpp
/// This is a constexpr, which contains multiple statements
constexpr int factorial_cpp14(int n) {
    int result = 1;
    for (int i = 1; i <= n; ++i) {
        result *= i;
    }
    return result;
}
```

# C++17

## `std::variant` (C++17)

`std::variant` is a C++17 feature that allows you to create a type-safe union.

Example:

```cpp
#include <variant>
#include <iostream>
std::variant<uint32_t, double> var;
var = uint32_t(10);
if (std::holds_alternative<uint32_t>(myVariant)) {
  std::cout << std::get<uint32_t>(var) << std::endl;
}
var = double(3.14);
if (std::holds_alternative<double>(myVariant)) {
  std::cout << std::get<double>(var) << std::endl;
}
```

The traditional union is not type-safe, and we don't know which type is stored currently. Moreover, traditional union does not support complex type, such as `std::string`.

## `constexpr` Lambda Expression (C++17)

The order is `[] { /* Code... */ }()`. See the sample:

```cpp
constexpr int TABLE_SIZE = 10;
constexpr auto table = [] {
  std::array<int, TABLE_SIZE> A = {};
  for (unsigned i = 0; i < TABLE_SIZE; i++) A[i] = i * i;
  return A;
}();
// static constexpr std::array<int, 10UL> table = {0, 1, 4, 9, 16, 25, 36, 49, 64, 81}
```

## Pack Expansion (C++17)

Pack expansion is a way to expand a parameter pack into individual template parameters when using them in expressions, function calls, or type aliases. This allows the compiler to process each element in the parameter pack.

Example:

```cpp
template <typename... Args>
void print_all(Args... args) {
    (std::cout << ... << args) << std::endl;  // Fold expression
}
print_all(1, 2, 3, "Hello", 4.5);  // prints: 123Hello4.5
```

## Fold Expression (C++17)

Fold expression is a way to perform operations on a parameter pack. It allows you to apply an operation to each element in the parameter pack and combine the results.

There are four types of fold expressions: 

1. Unary right fold `(pack op ...)`: `(pack[0] op (... op (pack[N-1] op pack[N])))`
2. Unary left fold `(... op pack)`: `(((pack[0] op pack[1]) op ...) op pack[N])`
3. Binary right fold `(pack op ... op init)`: `(pack[0] op (... op (pack[N-1] op init)))`
4. Binary left fold `(init op ... op pack)`: `(((init op pack[0]) op ...) op pack[N])`

* `op` - any of the following 32 binary operators: `+` `-` `*` `/` `%` `^` `&` `|` `=` `<` `>` `<<` `>>` `+=` `-=` `*=` `/=` `%=` `^=` `&=` `|=` `<<=` `>>=` `==` `!=` `<=` `>=` `&&` `||` `,` `.*` `->*`. In a binary fold, both ops must be the same.
* `pack` - an expression that contains an unexpanded pack and does not contain an operator with precedence lower than cast at the top level.
* `init` - an expression that does not contain an unexpanded pack and does not contain an operator with precedence lower than cast at the top level.

The comma operator `,` is usually used with a unary fold expression to achieve more complex operations, because `,` drops the result of the expressions, remaining only the last one. For instance:

```cpp
template<typename... Args>
void print_if_integral(Args... args) {
  // The following code uses a binary fold expression. However, there is no way to add a space between the arguments.
  (std::cout << ... << (std::is_integral_v<T> ? args : 0)) << std::endl;
  // The following code uses a unary fold expression with a comma operator.
  ((std::cout << (std::is_integral_v<T> ? args : 0) << " "), ...) << std::endl;
}
```

# C++20

## Designated Initializer (C++20)

Designated Initializer is a C++20 feature that allows you to initialize a struct with designated fields.

Sample code:

```cpp
class MyClass {
public:
  int a;
  double b;
};

MyClass my_class = {.a = 10, .b = 3.14};
```

## Concepts (C++20)

Concepts are a C++20 feature that allows you to define constraints on template parameters.

Define a `concept`:

```cpp
// Define a concept
template <typename T>
concept Incrementable = requires(T &t) {
  { ++t } -> std::same_as<T &>;
};
// Use the concept in a template
template <Incrementable T>
void increment(T &value) {
  ++value;
}
increment("Hello");  // Error: "Hello" is not Incrementable
```

Rules to write a concept. See this sample:

```cpp
template<typename Any> concept has_N = requires{ requires Any::N - Any::N == 0; };
template<typename A> concept fizz_c = has_N<A> && requires{ requires A::N % 3 == 0; };
template<typename A> concept buzz_c = has_N<A> && requires{ requires A::N % 5 == 0;};
template<typename A> concept fizzbuzz_c = fizz_c<A> && buzz_c<A>;
```

Use the `requires` directly in template:

```cpp
template <typename T>
  requires std::integral<T>
T add(T a, T b) {
  return a + b;
}
```

`requires` clause for SFINAE (Substitution Failure Is Not An Error).

```cpp
template <typename T>
  requires requires(T t) {
    { t + t } -> std::same_as<T>;
  }
T add_req(T a, T b) {
  return a + b;
}
```

There are some standard concepts defined in `<concepts>`:

* `std::integral` - Integral types
* `std::floating_point` - Floating point types
* `std::same_as` - Types that are the same as another type
* `std::convertible_to` - Types that can be converted to another type
* `std::invocable` - Types that can be called as a function
* `std::copyable` - Types that can be copied
* `std::movable` - Types that can be moved
* `std::default_initializable` - Types that can be default initialized
* `std::constructible_from` - Types that can be constructed from another type
* `std::assignable_from` - Types that can be assigned from another type
* `std::destructible` - Types that can be destructed
* `std::swappable` - Types that can be swapped
* `std::regular` - Types that are regular
* `std::totally_ordered` - Types that are totally ordered
* `std::semiregular` - Types that are semiregular
* `std::regular_invocable` - Types that are regular invocable
* `std::predicate` - Types that are predicate
* `std::relation` - Types that are relation
* `std::strict_weak_order` - Types that are strict weak order
* `std::totally_ordered_with` - Types that are totally ordered with another type

# C++23
