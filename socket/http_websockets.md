HTTP and WebSockets Note
===

# 1 Web Basics

**HTTP** (HyperText Transfer Protocol) is a protocol that allows the fetching of resources, such as HTML documents. It is a client-server protocol, in which the client makes a request to the server, and the server responds with the requested resource. It is a stateless protocol, meaning that the server does not keep any data between two requests. The client and server communicate by sending text messages. The messages are sent in plaintext, so they can be read by anyone who intercepts the messages.

**HTTPS** (HyperText Transfer Protocol Secure) is the secure version of HTTP. It uses encryption to secure the data transmitted between the client and the server.

**WebSockets** is a protocol that provides full-duplex communication channels over a single TCP connection. It is designed to be implemented in web browsers and web servers, but it can be used by any client or server application. The WebSocket protocol makes more interaction between a browser and a website possible, facilitating live content and the creation of *real-time application*.

These three protocols belong to the application layer of the TCP/IP model.

## 1.1 HTTP

A typical HTTP request consists of a **Request Line**, **Headers**, and a **Body** (optional). The request line contains the **Method**, the **URL**, and the **HTTP version**. The headers contain additional information about the request, such as the type of content being sent or accepted, authentication tokens, etc. The body contains the data being sent to the server.

Common HTTP methods are:

* **GET**: Requests data from the server.
* **POST**: Sends data to the server to create a new resource.
* **PUT**: Updates an existing resource with new data.
* **DELETE**: Deletes a resource from the server.
* **PATCH**: Partially updates a resource.
* **HEAD**: Retrieves metadata (headers) about a resource but not the resource itself.
* **OPTIONS**: Requests information about the communication options for a resource.

An HTTP response consists of a **Status Line**, **Headers**, and a **Body** (optional). The status line contains the **HTTP Version**, the **Status Code**, and a **Status Message**. The headers contain additional information about the response, such as the type of content being sent, the length of the content, etc. The body contains the data being sent back to the client.

Common **HTTP Status Codes** are:

* **100 Continue**: The server has received the request headers and the client should proceed to send the request body.
* **101 Switching Protocols**: The server has agreed to switch protocols.
* **102 Processing**: The server has received the request headers and the client should proceed to send the request body.
* **103 Early Hints**: The server has received the request headers and the client should proceed to send the request body.
* **200 OK**: The request was successful, and the server returned the requested data.
* **201 Created**: The resource was successfully created (typically in response to a POST request).
* **202 Accepted**: The request has been accepted for processing but has not been completed.
* **203 Non-Authoritative Information**: The response contains modified data from a third party.
* **204 No Content**: The request was successful, but there is no data to return.
* **206 Partial Content**: The server is delivering only part of the resource due to a range header sent by the client.
* **301 Moved Permanently**: The requested resource has been permanently moved to a new location.
* **302 Found (Moved Temporarily)**: The requested resource has been temporarily moved to a new location.
* **303 See Other**: The response to the request can be found under a different URI.
* **304 Not Modified**: The client can use cached data.
* **307 Temporary Redirect**: The requested resource has been temporarily moved to a new location.
* **308 Permanent Redirect**: The requested resource has been permanently moved to a new location.
* **400 Bad Request**: The request was malformed or invalid.
* **401 Unauthorized**: The client must authenticate to access the requested resource.
* **402 Payment Required**: Reserved for future use (originally intended for digital payment systems).
* **403 Forbidden**: The client does not have permission to access the resource.
* **404 Not Found**: The requested resource could not be found.
* **405 Method Not Allowed**: The HTTP method used is not supported for this resource.
* **406 Not Acceptable**: The server cannot produce a response matching the list of acceptable values.
* **407 Proxy Authentication Required**: Authentication with the proxy is required.
* **408 Request Timeout**: The server timed out waiting for the request.
* **409 Conflict**: The request conflicts with the current state of the server.
* **410 Gone**: The requested resource is no longer available and will not be available again.
* **411 Length Required**: The request did not specify the length of its content.
* **412 Precondition Failed**: Server does not meet one of the preconditions the client put on the request.
* **413 Payload Too Large**: The request is larger than the server is willing or able to process.
* **414 URI Too Long**: The URI provided was too long for the server to process.
* **415 Unsupported Media Type**: The media format of the requested data is not supported.
* **429 Too Many Requests**: The user has sent too many requests in a given amount of time.
* **500 Internal Server Error**: The server encountered an error processing the request.
* **501 Not Implemented**: The server does not support the functionality required to fulfill the request.
* **502 Bad Gateway**: The server received an invalid response from an upstream server.
* **503 Service Unavailable**: The server is temporarily unable to handle the request.
* **504 Gateway Timeout**: The server did not receive a timely response from an upstream server.
* **505 HTTP Version Not Supported**: The server does not support the HTTP protocol version used in the request.
* **508 Loop Detected**: The server detected an infinite loop while processing the request.
* **511 Network Authentication Required**: The client needs to authenticate to gain network access.

Request-Response flow example:

```vbnet
<!-- Client Request (GET) -->
GET /products HTTP/1.1
Host: api.example.com
Accept: application/json

<!-- Server Response (a json file) -->
HTTP/1.1 200 OK
Content-Type: application/json
Content-Length: 234

[
  { "id": 1, "name": "Product A", "price": 10 },
  { "id": 2, "name": "Product B", "price": 20 }
]
```

## 1.2 WebSocket

TODO


# 2 Commandline Tools

## 2.1 `curl` Command

`curl` is a command-line tool for transferring data with URLs. It supports various protocols, including HTTP, HTTPS, FTP, FTPS, and more. It is widely used to interact with web services and APIs from the command line.

> GPT said that `curl` is more advanced than `wget`.

Usage samples:

```shell
# Send a GET request to a URL and print the response to the console
curl http://example.com
# Use curl in verbose mode
curl -v http://example.com
# Save the response to a file
curl -o output.html http://example.com
# Print the response headers
curl -i http://example.com

# Send a GET request with a custom header
curl -H "Authorization: Bearer your_token" http://example.com
# Send a POST request with data, typically in the form of key=value pairs or JSON
curl -X POST http://example.com -d "key1=value1&key2=value2"
curl -X POST http://example.com -H "Content-Type: application/json" -d '{"name": "John", "email": "john@example.com"}'
# Send a PU request
curl -X PUT http://example.com -d '{"name": "Updated Name"}' -H "Content-Type: application/json"
# Send a DELETE request
curl -X DELETE http://example.com
# Follow redirects
curl -L http://google.com
# Send a request with basic authentication
curl -u username:password http://example.com/protected

# Save cookies to a file
curl -c cookies.txt http://example.com
# Send a request with cookies
curl -b cookies.txt http://example.com
```

## 2.2 `wscat` Command

Install `wscat` from npm:

```shell
npm install -g wscat
```

Connect to a WebSocket server:

```shell
wscat -c wss://echo.websocket.org
```

Options:

TODO

## 2.3 `websocat` Command

Install binary `websocat` from github:

```shell
curl -L -o websocat https://github.com/vi/websocat/releases/download/v1.8.0/websocat_amd64-linux
```

Usage:

```shell
websocat wss://echo.websocket.org
# Print the response headers
websocat -v wss://echo.websocket.org
```

# 3 UWebSockets

> UWebSockets only support WebSockets server end. To build an client end application, consider using other libraries.

UWebSockets, based on USockets, is a head-only WebSocket/HTTP library for C++. It supports server mode and provides a simple API for sending and receiving messages.

Server code sample:

```cpp
#include <App.h>
int main() {
  uWS::App app = uWS::App()
    // Add a WebSocket endpoint at ws://localhost:9001
    .ws<PerSocketData>(
        "/*",
        {.open =
              [](uWS::WebSocket<false, true, PerSocketData> *ws) {
                std::cout << "[SERVER] WebSocket connection opened!" << std::endl;
              },
          .message =
              [](uWS::WebSocket<false, true, PerSocketData> *ws, std::string_view message, uWS::OpCode op_code) {
                std::cout << "[SERVER] Received message: " << message << std::endl;
                /// Do something...
                if (message == "get_status") {
                  ws->send("status: OK");
                } else {
                  ws->send("task: " + std::string(message) + " completed");
                }
              },
          .close =
              [](uWS::WebSocket<false, true, PerSocketData> *ws, int code, std::string_view message) {
                std::cout << "[SERVER] WebSocket connection closed!" << std::endl;
              }})
    // Add an HTTP endpoint at http://localhost:9001
    .get("/",
          [](uWS::HttpResponse<false> *res, uWS::HttpRequest *req) {
            std::cout << "[SERVER] HTTP request received!" << req->getUrl() << std::endl;
            res->end("<h1>Hello world!</h1>");
          })
    // Add another HTTP endpoint at http://localhost:9001/status
    .get("/status",
          [](uWS::HttpResponse<false> *res, uWS::HttpRequest *req) {
            std::cout << "[SERVER] HTTP request received!" << req->getUrl() << std::endl;
            res->end("status: OK");
          })
    // Listen on port 9001
    .listen(9001,
            [](auto *token) {
              if (token) {
                std::cout << "[SERVER] Listening on port " << 9001 << std::endl;
              } else {
                std::cout << "[SERVER] Failed to listen on port " << 9001 << std::endl;
              }
            })
    // Start the loop
    .run();
  return 0;
}
```


