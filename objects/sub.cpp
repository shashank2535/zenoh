
#include "zenoh.hxx"
#include <iostream>
#include "getargs.hxx"

using namespace zenoh;
using namespace zenoh::ext;  // For serialize/deserialize functions

struct CustomStruct {
    std::vector<float> pos;
    std::vector<float> gyro;


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

bool __zenoh_deserialize_with_deserializer(zenoh::ext::Deserializer &deserializer, CustomStruct &s, ZResult *err) {
        return zenoh::ext::detail::deserialize_with_deserializer(deserializer, s.pos, err) &&
           zenoh::ext::detail::deserialize_with_deserializer(deserializer, s.gyro, err);
}

int main(int argc, char **argv) {

        CustomStruct s;
   std::cout<< "Initilaizing..." << std::endl;
   // Config config = Config::create_default();
      auto &&[config, args] =ConfigCliArgParser(argc, argv).run();

   auto session = Session::open(std::move(config));
   std::cout<< "Session starts..." << std::endl;

   auto subscriber = session.declare_subscriber(
    KeyExpr("demo/example/simple"),
      [](const Sample& sample) 
      {
        CustomStruct s = ext::deserialize<CustomStruct>(sample.get_payload());

                // Print the received data
                std::cout << "Received data:" << std::endl;
                s.print();
               },
      closures::none
   );
   // Wait for a key press to exit
   char c = getchar();
}
   // Wait for a key press to exit
  
/* zenoh::ext::Deserializer deserializer(sample.get_payload());
           std::vector<float> received_data = deserializer.deserialize<std::vector<float>>();
       std::cout << "Received vector: ";
                for (float value : received_data) {
                    std::cout << value << " ";*/