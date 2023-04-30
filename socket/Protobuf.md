# Google Protobuf 入门

Protobuf是一种语言无关、平台无关、可扩展的序列化结构数据的方法，它可用于数据通信协议、数据存储等，具有空间开销小、解析速度快、兼容性好等优点。

> 其他协议工具JSON、XML。json\xml都是基于文本格式，protobuf是二进制格式。

安装：

```shell
git clone https://github.com/protocolbuffers/protobuf.git
cd protobuf
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make -j12
sudo make install
```

Protobuf的**消息**和**服务**在`.proto`文件中定义。通过proto编译器编译，生成不同语言的类库。类中提供了提供了从文件和流中检索数据、从数据中提取单个值、检查数据是否存在、将数据序列化回文件或流以及其他有用功能的实用方法。

Protobuf支持在不改动现有消息的前提下，在`.proto`中添加或删除数据成员。

Protobuf局限：

1. Protobuf假设消息可以被完整地放入内存（小与几个MB）

2. 传输带有大量浮点数组的工程数据时，效率较低。

Protobuf使用流程：

![](image/protocol-buffers-concepts.png)

# .proto文件

`.proto`文件的例子如下：

```js
syntax = "proto2";

package tutorial;

message Person {
  optional string name = 1;
  optional int32 id = 2;
  optional string email = 3;

  enum PhoneType {
    MOBILE = 0;
    HOME = 1;
    WORK = 2;
  }

  message PhoneNumber {
    optional string number = 1;
    optional PhoneType type = 2 [default = HOME];
  }

  repeated PhoneNumber phones = 4;
}

message AddressBook {
  repeated Person people = 1;
}
```

`package`项目名用于区分不同proto项目，生成的C++类会位于这个命令空间下。

`message`是包含一组类型字段的集合。许多简单的数据类型，如bool, int32, float, double, string都可以作为字段类型。可以用已经定义的消息做字段类型。可以定义嵌套在其他消息中的消息类型。

`= 1`、`= 2`确定了每个元素唯一的字段号。1-15号字段编码时比其他字段少用一个字节。因此低序号字段最好留给常用的元素。重复字段需要重编码字段号，因此适合放在低位。

每个字段都必须用下面三种修饰词之一修饰：

1. `optional` 字段可以被设置，也可以不被设置。若没有被设置，使用默认值，用户可以为简单的数据设定默认值。

2. `repeated` 字段可以被重复多次（包括0次）。可视作动态数组。

3. `required` 字段的元素必须被初始化。最好不要用这个修饰词。

生成代码中的成员方法距离：

```cpp
    // string name
    inline bool has_name() const;
    inline void clear_name();
    inline const ::std::string& name() const;
    inline void set_name(const ::std::string& value);
    inline void set_name(const char* value);
    inline ::std::string* mutable_name();

    // int32 id
    inline bool has_id() const;
    inline void clear_id();
    inline int32_t id() const;
    inline void set_id(int32_t value);

    // repeated PhoneNumber phones
    inline int phones_size() const;
    inline void clear_phones();
    inline const ::google::protobuf::RepeatedPtrField< ::tutorial::Person_PhoneNumber >& phones() const;
    inline ::google::protobuf::RepeatedPtrField< ::tutorial::Person_PhoneNumber >* mutable_phones();
    inline const ::tutorial::Person_PhoneNumber& phones(int index) const;
    inline ::tutorial::Person_PhoneNumber* mutable_phones(int index);
    inline ::tutorial::Person_PhoneNumber* add_phones();
```

生成代码中的类对每一个字段提供了get和set方法，以及判有函数和清空函数。

对repeated类型字段提供了get_size方法、下标索引方法。

标准消息方法：

* `bool IsInitialized() const;` // checks if all the required fields have been set.

* `string DebugString() const;` // returns a human-readable representation of the message, particularly useful for debugging.

* `void CopyFrom(const Person& from);` overwrites the message with the given message’s values.

* `void Clear();` clears all the elements back to the empty state.

解析和序列化：

* `bool SerializeToString(string* output) const;` serializes the message and stores the bytes in the given string. Note that the bytes are binary, not text; we only use the string class as a convenient container.

* `bool ParseFromString(const string& data);` parses a message from the given string.

* `bool SerializeToOstream(ostream* output) const;` writes the message to the given C++ ostream.

* `bool ParseFromIstream(istream* input);` parses a message from the given C++ istream.

cmake编译：

```cmake
find_package(protobuf REQUIRED)
add_library(messagelib src/messagelib.pb.cc)
add_executable(target src/target.cpp)
target_link_libraries(target messagelib protobuf::libprotobuf)
```
