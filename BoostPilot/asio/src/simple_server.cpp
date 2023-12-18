#include <boost/asio.hpp>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

// a multi-threaded sever that handles incoming connections from clients,
// listens for incoming messages from the connected clients, and dispatches the
// messages to a pool of worker trheads for processing
class Connection : public std::enable_shared_from_this<Connection> {
public:
    Connection(boost::asio::io_service&     ios,
               boost::asio::ip::tcp::socket socket)
        : socket_(std::move(socket)) {}

    boost::asio::ip::tcp::socket& socket() {
        return socket_;
    }

    void start() {
        do_read();
    }

private:
    void do_read() {
        auto self(shared_from_this());
        socket_.async_read_some(boost::asio::buffer(data_),
                                [this, self](boost::system::error_code ec,
                                             std::size_t bytes_transferred) {
                                    if (!ec) {
                                        do_process(bytes_transferred);
                                        do_write(bytes_transferred);
                                    }
                                });
    }

    void do_process(std::size_t length) {
        std::string message(data_, length);
        std::cout << "Received message: " << message << std::endl;

        // do some processing on the received message
        std::transform(message.begin(), message.end(), message.begin(),
                       ::toupper);

        message_ = message;
    }

    void do_write(std::size_t length) {
        auto self(shared_from_this());
        boost::asio::async_write(
            socket_, boost::asio::buffer(message_),
            [this, self](boost::system::error_code ec,
                         std::size_t /*bytes_transferred*/) {
                if (!ec) {
                    do_read();
                }
            });
    }

private:
    boost::asio::ip::tcp::socket socket_;
    enum { max_length = 1024 };
    char        data_[max_length];
    std::string message_;
};

class Server {
public:
    Server(boost::asio::io_service& ios, short port)
        : ios_(ios),
          acceptor_(ios, boost::asio::ip::tcp::endpoint(
                             boost::asio::ip::tcp::v4(), port)),
          socket_(ios) {
        do_accept();
    }

private:
    void do_accept() {
        acceptor_.async_accept(socket_, [this](boost::system::error_code ec) {
            if (!ec) {
                std::make_shared<Connection>(ios_, std::move(socket_))->start();
            }
            do_accept();
        });
    }

private:
    boost::asio::io_service&       ios_;
    boost::asio::ip::tcp::acceptor acceptor_;
    boost::asio::ip::tcp::socket   socket_;
};

int main(int argc, char* argv[]) {
    try {
        if (argc != 2) {
            std::cerr << "Usage: server <port>\n";
            return 1;
        }

        boost::asio::io_service ios;

        Server server(ios, std::atoi(argv[1]));

        std::vector<std::thread> threads;
        for (int i = 0; i < 4; ++i) {
            threads.emplace_back([&ios]() { ios.run(); });
        }

        for (auto& t : threads) {
            t.join();
        }
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}
