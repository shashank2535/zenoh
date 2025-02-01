#include "zenoh.hxx"

#include <thread> // For sleep
#include <chrono> // For time duration
#include "getargs.hxx"
#include <sstream> // For stringstream
#include <vector> // For std::vector

using namespace zenoh;

int main(int argc, char **argv) {
    std::cout << "Initializing..." << std::endl;

    // Create default configuration and open session
    // Config config = Config::create_default();
    // auto session = Session::open(std::move(config));
        auto &&[config, args] =
        ConfigCliArgParser(argc, argv)
                    .run();
    auto session = Session::open(std::move(config));
    std::cout << "Session started..." << std::endl;

    // Create a Publisher object
    auto publisher = session.declare_publisher(KeyExpr("demo/example/simple"));
    std::cout << "Publisher declared..." << std::endl;
  
    for (int i = 0; i < 10; ++i) { // Loop to publish 10 batches
        // Prepare a batch of 5 integers
        std::vector<int> batch;
        for (int j = 0; j < 5; ++j) {
            batch.push_back(i * 5 + j);
        }
        // Serialize the vector into raw bytes
        //std::vector<uint8_t> serialized_data(batch.size() * sizeof(int));
        //std::memcpy(serialized_data.data(), batch.data(), serialized_data.size());

      // publisher.put(serialized_data);
      //  std::cout << "Published vector: ";
        //for (int val : batch) {
            //std::cout << val << " ";
            zenoh::ext::Serializer  serializer;
        serializer.serialize(batch);
        Bytes serialized_data = std::move(serializer).finish();

        // Publish the serialized data
        publisher.put(std::move(serialized_data));
        std::cout << "Published vector: ";
        for (const auto& val : batch) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
        }
       
         std::this_thread::sleep_for(std::chrono::seconds(1));
}
    
/*float array[50];
    for (int i = 0; i < 50; ++i) {
        array[i] = (i + 1) * 1.5f; 
         }


    for (int i = 0; i < 50; i += 5) { // Example: Publish 50 messages in total, 5 per iteration
            std::vector<float> message_batch;

            // Generate 5 random float values and store them in message_batch
            for (int j = 0; j < 5; ++j) 
            {
                // Push random float
                 message_batch.push_back(array[i + j]);   
                 
            }
         size_t batch_size = message_batch.size() * sizeof(float);
        std::vector<uint8_t> serialized_data(batch_size);
        std::memcpy(serialized_data.data(), message_batch.data(), batch_size);

        // Convert serialized data into zenoh::Bytes
        zenoh::Bytes message_bytes(std::move(serialized_data));

        publisher.put(std::move(serialized_data));
        std::cout << "Published batch of floats: ";
        for (float value : message_batch)
         {
            std::cout << value << " ";
        }
        std::cout << std::endl;

    }
}

        /*Convert the batch to a string and publish it
       
         std::stringstream message_stream;
        for (size_t k = 0; k < message_batch.size(); ++k) {
            message_stream << message_batch[k];
            if (k < message_batch.size() - 1) {
                message_stream << " "; // Separate floats with space
            }
        }
        std::string message_str = message_stream.str();
        std::cout << "Published: " <  // Get the final string
        publisher.put(std::move(message_str));*/
      //  std::cout << "Published: " << message_str << std::endl;
    
       /* for (size_t k = 0; k < message_batch.size(); ++k) {
            message_stream << message_batch[k];
            if (k < message_batch.size() - 1) {
                message_stream << " "; 
                publisher.put(message_batch);// Separate the floats with a space

                 std::cout << "Published"<<message_stream.str() << std::endl;
                 std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }*/
         
        

   //  // Properly close the session
  // session.close();
    // return 0;