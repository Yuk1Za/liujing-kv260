#pragma once
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cctype>
namespace lj2 {
struct Response {int code=200;std::string type="application/json",body;};
class Http {
 int fd_=-1;std::atomic<bool>stop_{false};std::thread thread_;
public:
 Http(int port,std::function<Response(const std::string&,const std::string&,const std::string&)> handle){
  fd_=socket(AF_INET,SOCK_STREAM,0);if(fd_<0)throw std::runtime_error("HTTP socket");int yes=1;setsockopt(fd_,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof yes);
  sockaddr_in addr{};addr.sin_family=AF_INET;addr.sin_addr.s_addr=htonl(INADDR_LOOPBACK);addr.sin_port=htons(port);
  if(bind(fd_,(sockaddr*)&addr,sizeof addr)||listen(fd_,8)){close(fd_);throw std::runtime_error("HTTP bind/listen");}
  thread_=std::thread([this,handle]{while(!stop_){int c=accept(fd_,nullptr,nullptr);if(c<0)continue;timeval timeout{0,200000};setsockopt(c,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof timeout);setsockopt(c,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof timeout);
   std::string data;char buf[4096];size_t header=std::string::npos,length=0;bool valid=false;
   while(data.size()<16384){int n=recv(c,buf,sizeof buf,0);if(n<=0)break;data.append(buf,n);header=data.find("\r\n\r\n");if(header==std::string::npos)continue;
    std::string headers=data.substr(0,header);std::transform(headers.begin(),headers.end(),headers.begin(),[](unsigned char c){return std::tolower(c);});
    if(headers.find("transfer-encoding:")!=std::string::npos)break;
    auto pos=headers.find("content-length:");if(pos!=std::string::npos){try{length=std::stoul(headers.substr(pos+15));}catch(...){break;}}
    if(length>8192)break;if(data.size()>=header+4+length){valid=true;break;}}
   Response r;if(valid){std::istringstream line(data);std::string method,path;line>>method>>path;
    try{r=handle(method,path,data.substr(header+4,length));}catch(const std::exception&){r={400,"application/json","{\"error\":\"invalid request\"}"};}}
   else r={400,"application/json","{\"error\":\"bounded request required\"}"};
   std::string head="HTTP/1.1 "+std::to_string(r.code)+" Reply\r\nContent-Type: "+r.type+"\r\nContent-Length: "+std::to_string(r.body.size())+"\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
   auto sendall=[&](const std::string&s){size_t at=0;while(at<s.size()){auto n=send(c,s.data()+at,s.size()-at,MSG_NOSIGNAL);if(n<=0)return false;at+=n;}return true;};if(sendall(head))sendall(r.body);close(c);
  }});
 }
 ~Http(){stop_=true;shutdown(fd_,SHUT_RDWR);close(fd_);if(thread_.joinable())thread_.join();}
};
}
