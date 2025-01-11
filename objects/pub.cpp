/*#include "zenoh.hxx"
#include <thread> // For sleep
#include <chrono> // For time duration
#include "getargs.hxx"
#include <sstream> // For stringstream
#include <vector> // For std::vector
#include <typeinfo> 
#include <iostream>
using namespace zenoh;

struct imu
    {
    public:
      std:: vector<float> pos;
      std::vector<float>gyro;

    // Default constructor
    imu() {
        pos.resize(3, 0.0f); // Resize the vector to 3 elements and set initial value to 0.0f
        gyro.resize(3, 0.0f); // Resize the vector to 3 elements and set initial value to 0.0f
    }
    

    void update(int iteration) {
        for (int i = 0; i < 3; ++i) {
            pos[i] = iteration * 1.0f + i;  // Example update logic
            gyro[i] = iteration * 2.0f + i; // Example update logic
        }
    }
    void print() const {
        std::cout << "pos: ";
        for (float p : pos) {
            std::cout << p << " ";
        }
        std::cout << std::endl;

        std::cout << "gyro: ";
        for (float g : gyro) {
            std::cout << g << " ";
        }
        std::cout << std::endl;
    }
    
    };
    bool __zenoh_serialize_with_serializer(zenoh::ext::Serializer& serializer, imu& s, ZResult* err) {
         return zenoh::ext::detail::serialize_with_serializer(serializer, s.pos, err) &&
          zenoh::ext::detail::serialize_with_serializer(serializer, s.gyro, err);
    }
int main(int argc, char **argv)
{
    imu s;  // Create an object of imu

    std::cout << "Initializing..." << std::endl;
    auto &&[config, args] = ConfigCliArgParser(argc, argv).run();
    auto session = Session::open(std::move(config));
    std::cout << "Session started..." << std::endl;

    // Create a Publisher object
    auto publisher = session.declare_publisher(KeyExpr("demo/example/simple"));
    std::cout << "Publisher declared..." << std::endl;

    
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
*/
        
        /*    zenoh::ext::Serializer  serializer;
        serializer.serialize(batch);
        Bytes serialized_data = std::move(serializer).finish();

        // Publish the serialized data
        publisher.put(std::move(serialized_data));
        std::cout << "Published vector: ";
        for (const auto& val : batch) {
            std::cout << val << " ";
            std::cout << "The type of message_stream is: " << typeid(val).name() << std::endl;

        }
        std::cout << std::endl;
        }
         
         std::this_thread::sleep_for(std::chrono::seconds(1));
}
*/
    
#include "zenoh.hxx"
#include <thread> // For sleep
#include <chrono> // For time duration
#include "getargs.hxx"
#include <sstream> // For stringstream
#include <vector> // For std::vector
#include <typeinfo> 
#include <iostream>
using namespace zenoh;

struct CustomStruct {
  std::vector<float> pos;
  std::vector<float> gyro;
 
 CustomStruct() : pos(3, 0.0f), gyro(3, 0.0f) {}

  void update(int iteration) {

        for (int j = 0; j < 3; ++j) {

            pos[j] = iteration * 1.0f + j;  // Example update logic
            gyro[j] = iteration * 2.0f + j; // Example update logic
        }
    }
  void print() const 
  {
        std::cout << "pos: ";
        for (float p : pos) {
            std::cout << p << " ";
        }
        std::cout << std::endl;

        std::cout << "gyro: ";
        for (float g : gyro) {
            std::cout << g << " ";
        }
        std::cout << std::endl;
    }
};

// One needs to implement __zenoh_serialize_with_serializer and __zenoh_deserialize_with_deserializer
// in the same namespace, where CustomStruct is defined.
// To simplify implementation users are allowed to use
// serialize_with_serializer and deserialize_with_deserializer functions defined in zenoh::ext::detail namespace.
bool __zenoh_serialize_with_serializer(zenoh::ext::Serializer& serializer, const CustomStruct& s1, ZResult* err) {
  return zenoh::ext::detail::serialize_with_serializer(serializer, s1.pos, err)&&
  zenoh::ext::detail::serialize_with_serializer(serializer, s1.gyro, err);
}

//bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer& deserializer, CustomStruct& s, ZResult* err) {
 // return zenoh::ext::detail::deserialize_with_deserializer(deserializer, s.vd, err) ;}

int main(int argc, char **argv) {
    CustomStruct s1;
      //std::cout << "The type of message_stream is: " << typeid(val).name() << std::endl;
    std::cout << "Initializing..." << std::endl;
    auto &&[config, args] = ConfigCliArgParser(argc, argv).run();
    auto session = Session::open(std::move(config));
    std::cout << "Session started..." << std::endl;

    // Create a Publisher object
    auto publisher = session.declare_publisher(KeyExpr("demo/example/simple"));
    std::cout << "Publisher declared..." << std::endl;
    for(int i=0;i<5;i++)
    {
        std::cout << "Iteration " << i + 1 << ":" << std::endl;
        s1.update(i);
        s1.print();
        Bytes b = ext::serialize(s1);
        publisher.put(std::move(b));
    }
  
 
 // CustomStruct s_out = ext::deserialize<CustomStruct>(b);
}