#include "zenoh.hxx"
#include <string>
#include <iostream>
#include <thread>
#include<array>
#include<cstdint>
#include "getargs.hxx"
//#include <zenoh.hpp>
using namespace zenoh;
using namespace zenoh::ext;

typedef std::array<int, 4> int4;
typedef std::array<double, 2> vec2;
typedef std::array<double, 3> vec3;
typedef std::array<double, 4> vec4;
typedef std::array<double, 6> vec6;
typedef std::array<double, 7> vec7;
typedef std::array<double, 12> vec12;
typedef std::array<double, 18> vec18;
typedef std::array<double, 19> vec19;
typedef std::array<std::array<double, 3>, 3> mat3x3;
typedef std::array<std::array<double, 4>, 4> mat4x4;
typedef std::array<std::array<double, 6>, 6> mat6x6;
typedef std::array<std::array<double, 8>, 8> mat8x8;
typedef std::array<std::array<double, 12>, 12> mat12x12;
typedef std::array<std::array<double, 12>, 18> mat12x18;
typedef std::array<std::array<double, 18>, 18> mat18x18;
typedef std::array<std::array<double, 6>, 12> mat6x12;
typedef std::array<std::array<double, 12>, 6> mat12x6;
typedef std::array<std::array<double, 6>, 18> mat6x18;
typedef std::array<std::array<double, 18>, 6> mat18x6;

template <typename T, size_t N>
std::ostream &operator<<(std::ostream &os, const std::array<T, N> &arr) {
    os << "[";
    for (size_t i = 0; i < N; ++i) {
        os << arr[i];
        if (i != N - 1) os << ", ";
    }
    os << "]";
    return os;
}
// Function to print a 2D array
template <typename T, size_t R, size_t C>
std::ostream &operator<<(std::ostream &os, const std::array<std::array<T, C>, R> &arr) {
    os << "[";
    for (const auto &row : arr) {
        os << row << ", ";
    }
    os << "]";
    return os;
}
struct QuadrupedEstimationData {
    vec3 rB;
    vec3 vB;
    vec12 rP;
    vec12 vP;
    int4 cs;
    vec4 pc;
    int4 is_slipping;
    vec19 js;
    vec18 jv;
    vec18 ja;
    vec3 np;
    vec6 bw;
    vec3 obstacle_coords;

     friend std::ostream &operator<<(std::ostream &os, const QuadrupedEstimationData &s2) {
        os << "rB: " << s2.rB << ", vB: " << s2.vB << ", rP: " << s2.rP
           << ", vP: " << s2.vP << ", cs: " << s2.cs << ", pc: " << s2.pc
           << ", is_slipping: " << s2.is_slipping << ", js: " << s2.js
           << ", jv: " << s2.jv << ", ja: " << s2.ja << ", np: " << s2.np
           << ", bw: " << s2.bw << ", obstacle_coords: " << s2.obstacle_coords;
        return os;
    }
} ;

bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer& deserializer, QuadrupedEstimationData& s2, ZResult* err) {
return  zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.rB, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.vB, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.rP, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.vP, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.cs, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.pc, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.is_slipping, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.js, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.jv, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.ja, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.np, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.bw, err)&&
        zenoh::ext::detail::deserialize_with_deserializer(deserializer, s2.obstacle_coords, err);
}
int main(int argc, char **argv) {
      //std::cout << "The type of message_stream is: " << typeid(val).name() << std::endl;
    QuadrupedEstimationData s2;
    // Create a Publisher object
    std::cout<< "Initilaizing..." << std::endl;
   // Config config = Config::create_default();
      auto &&[config, args] =ConfigCliArgParser(argc, argv).run();

   auto session = Session::open(std::move(config));
   std::cout<< "Session starts..." << std::endl;
   auto subscriber = session.declare_subscriber(KeyExpr("demo/example/simple"),[](const Sample& sample) 
      {
        try
      {
        /* code */
     
            std::cout << "Received data:" << std::endl;
            QuadrupedEstimationData s2 = ext::deserialize<QuadrupedEstimationData >(sample.get_payload());
            std::cout << "QuadrupedEstimationData: " << s2 << std::endl;
      }
          catch (const std::exception &e) {
        std::cerr << "Error during deserialization: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error during deserialization." << std::endl;
    }
      },
      closures::none
   );
   // Wait for a key press to exit
   char c = getchar();

}