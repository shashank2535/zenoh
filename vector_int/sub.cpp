
#include "zenoh.hxx"
#include <iostream>
#include "getargs.hxx"

using namespace zenoh;
using namespace zenoh::ext;  // For serialize/deserialize functions

int main(int argc, char **argv) {
   std::cout<< "Initilaizing..." << std::endl;
   // Config config = Config::create_default();
      auto &&[config, args] =ConfigCliArgParser(argc, argv).run();

   auto session = Session::open(std::move(config));
   std::cout<< "Session starts..." << std::endl;

   auto subscriber = session.declare_subscriber(
      KeyExpr("demo/example/simple"),
      [](const Sample& sample) 
      {
                // Deserialize the data into a vector of integers
         zenoh::ext::Deserializer deserializer(sample.get_payload());
           std::vector<int> received_data = deserializer.deserialize<std::vector<int>>();
       //std::cout << "Received: " << sample.get_payload().as_string() << std::endl;
       std::cout << "Received vector: ";
                for (int value : received_data) {
                    std::cout << value << " ";
      }
               std::cout << std::endl;
               },
      closures::none
   );
   // Wait for a key press to exit
   char c = getchar();
}
