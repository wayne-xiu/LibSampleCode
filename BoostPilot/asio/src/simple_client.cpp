#include <boost/asio.hpp>
#include <iostream>
#include <string>

// simple server-client demo using boost::asio
class Client {
public:
    Client(boost::asio::io_service &ios, const std::string &server, const std::string &port)
        : ios_(ios), socket_(ios) {
            boost::asio::ip::tcp::resolver resovler(ios_);
            boost::asio::ip::tcp::resolver::results_type endpoints = resovler.resolve(server, port);
            boost::asio::connect(socket_, endpoints);
        }

    void sendMessage(const std::string &message) {
        boost::asio::write(socket_, boost::asio::buffer(message));
        // std::cout << "Sent message: " << message << std::endl;
    }

    std::string receiveMessage() {
        boost::asio::streambuf buf;
        boost::asio::read_until(socket_, buf, '\n');
        std::istream is(&buf);
        std::string message;
        std::getline(is, message);
        return message;
    }

private:
    boost::asio::io_service &ios_;
    boost::asio::ip::tcp::socket socket_;
};

int main(int argc, char *argv[]) {
    try {
        if (argc != 4) {
            std::cerr << "Usage: client <server> <port> <message>\n";
            return 1;
        }

        boost::asio::io_service ios;
        Client client(ios, argv[1], argv[2]);

        std::string message = argv[3];
        client.sendMessage(message + '\n');
        std::cout << "Sent message: " << message << std::endl;

        std::string response = client.receiveMessage();
        std::cout << "Received response: " << response << std::endl;
    }
    catch (std::exception &e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}