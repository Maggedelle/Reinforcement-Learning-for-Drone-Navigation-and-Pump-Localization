#include "HTTPRequest.hpp"
#include <iostream>
#include <random>
#ifdef __cplusplus
extern "C"
{      // tells C++ compiler to use C symbol name mangling (C compiler ignores)
#endif // __cplusplus
    double get_position_x();
    double get_position_y();
    void shutdown_drone();
    int move_drone(int x, int y);
    int get_random_number(int min, int max);
#ifdef __cplusplus
} // end of "C" symbol name mangling
#endif // __cplusplus

double get_position_x()
{
    try
    {
        http::Request request{"http://127.0.0.1:5000/get_position_x"};
        // send a get request
        const auto response = request.send("GET");
        // you can pass http::InternetProtocol::V6 to Request to make an IPv6 request
        auto result = std::string{response.body.begin(), response.body.end()};
        std::cerr << result << std::endl;
        auto output = std::stod(result);
        return output;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Request failed, error: " << e.what() << '\n';
    }
    return 0.0;
}

double get_position_y()
{
    try
    {
        http::Request request{"http://127.0.0.1:5000/get_position_y"};
        const auto response = request.send("GET");
        // you can pass http::InternetProtocol::V6 to Request to make an IPv6 request

        // send a get request
        auto result = std::string{response.body.begin(), response.body.end()};
        std::cerr << result << std::endl;
        auto output = std::stod(result);
        return output;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Request failed, error: " << e.what() << '\n';
    }
    return 0.0;
}

int move_drone(int x, int y) 
{
    try
    {
        // you can pass http::InternetProtocol::V6 to Request to make an IPv6 request
        http::Request request{"http://127.0.0.1:5000/move_drone?x=" + std::to_string(x) + "&y=" + std::to_string(y)};
        // send a get request
        const auto response = request.send("GET");
        auto result = std::string{response.body.begin(), response.body.end()};
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Request failed, error: " << e.what() << '\n';
    }

    return 0;
}

int get_random_number(int min, int max) {
    std::random_device rd;     // Only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // Random-number engine used (Mersenne-Twister in this case)
    std::uniform_int_distribution<int> uni(min,max); // Guaranteed unbiased

    return uni(rng);   
} 

void shutdown_drone()
{
    try
    {
        // you can pass http::InternetProtocol::V6 to Request to make an IPv6 request
        http::Request request{"http://127.0.0.1:5000/shutdown_drone"};

        // send a get request
        const auto response = request.send("GET");
    }
    catch (const std::exception &e)
    {
        std::cerr << "Request failed, error: " << e.what() << '\n';
    }
}
