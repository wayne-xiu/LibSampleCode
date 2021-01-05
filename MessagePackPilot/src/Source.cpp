#include <msgpack.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <string_view>
#include <set>

class Book final
{
public:
    int id;
    std::string title;
    std::set<std::string> tags;

public:
    MSGPACK_DEFINE(id, title, tags); // macro for serializaiton
};

int main()
{
    msgpack::type::tuple<int, bool, std::string> src(1, true, "example");

    // serialize the object into the buffer.
    // any classes that implements write(const char*,size_t) can be a buffer.
    std::stringstream buffer;
    msgpack::pack(buffer, src);

    // send the buffer ...
    buffer.seekg(0);

    // deserialize the buffer into msgpack::object instance.
    std::string str(buffer.str());

    msgpack::object_handle oh =
        msgpack::unpack(str.data(), str.size());

    // deserialized object is valid during the msgpack::object_handle instance is alive.
    msgpack::object deserialized = oh.get();

    // msgpack::object supports ostream.
    std::cout << deserialized << std::endl;

    // convert msgpack::object instance into the original type.
    // if the type is mismatched, it throws msgpack::type_error exception.
    msgpack::type::tuple<int, bool, std::string> dst;
    deserialized.convert(dst);

    // or create the new instance
    msgpack::type::tuple<int, bool, std::string> dst2 =
        deserialized.as<msgpack::type::tuple<int, bool, std::string>>();

    std::vector<int> v = {1, 2, 3, 4, 5};
    msgpack::sbuffer sbuf;                 // output buffer
    msgpack::pack(sbuf, v);                // serialization
    std::cout << sbuf.size() << std::endl; // check data length after serialization

    auto handle = msgpack::unpack(sbuf.data(), sbuf.size());
    auto obj = handle.get();
    std::cout << obj << std::endl;

    // the original data type needs to be known for converstion back
    std::vector<int> v2;
    obj.convert(v2);
    assert(std::equal(std::begin(v), std::end(v), std::begin(v2)));

    // packer for continuous serialization
    msgpack::sbuffer sbufs;
    msgpack::packer<decltype(sbufs)> packer(sbufs);

    using namespace std::string_literals; // need C++14
    packer.pack(10).pack("mondao"s).pack(std::vector<int>{1, 2, 3});

    for (decltype(sbufs.size()) offset = 0; offset != sbufs.size();)
    {
        auto handle = msgpack::unpack(sbufs.data(), sbufs.size(), offset);
        auto obj2 = handle.get();
    }

    // pack(), unpack() like JSON
    Book book1 = {1, "1984", {"a", "b"}}; // user defined type
    msgpack::sbuffer sbufb;               // output buffer
    msgpack::pack(sbufb, book1);          // serialize
            Book book2;
    try
    {
        auto objb = msgpack::unpack( // deserialize
                        sbufb.data(), sbufb.size())
                        .get();
        objb.convert(book2); // conversion, this failed?
    }
    catch (std::exception &e)
    {
        std::cout << e.what() << std::endl;
    }
    assert(book2.id == book1.id);
    assert(book2.tags.size() == 2);
    std::cout << book2.title << std::endl;

    return 0;
}