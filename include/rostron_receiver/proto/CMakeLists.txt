cmake_minimum_required(VERSION 3.16.2)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  vision/ssl_vision_detection.proto
  vision/ssl_vision_geometry.proto
  vision/ssl_vision_wrapper.proto 

  gc/ssl_gc_common.proto
  gc/ssl_gc_geometry.proto
  gc/ssl_gc_game_event.proto
  gc/ssl_gc_referee_message.proto
)

add_library(sslproto
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)

add_library(NAELIC::Proto ALIAS sslproto)

target_include_directories(sslproto PUBLIC
    ${Protobuf__INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(sslproto PUBLIC ${Protobuf_LIBRARIES})