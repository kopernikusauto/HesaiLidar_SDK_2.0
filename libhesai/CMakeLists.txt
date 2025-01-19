# ----------------------------------------#
# Hesai Tech <http://www.hesaitech.com>  #
# ----------------------------------------#

add_library(HesaiLidar
    src/plat_utils.cc
    src/auto_tick_count.cc
    Container/src/ring_2d_shared.cc
    Container/src/ring_2dex.cc
)
target_include_directories(HesaiLidar PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Lidar>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/UdpProtocol>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/UdpParser/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/UdpParser/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Container/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Container/src>

    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/Lidar>
    $<INSTALL_INTERFACE:include/UdpProtocol>
    $<INSTALL_INTERFACE:include/UdpParser>
    $<INSTALL_INTERFACE:include/Container>
)
set_target_properties(HesaiLidar PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
file(GLOB_RECURSE _container_headers
    Container/include/*.h
    Container/src/*.cc
)
file(GLOB_RECURSE _udpprotocol_headers
    UdpProtocol/*.h
)
file(GLOB_RECURSE _udpparser_headers
    UdpParser/include/*.h
    UdpParser/src/*.cc
)
file(GLOB_RECURSE _lidar_headers
    Lidar/*.h
)

add_library(HesaiPtcParser
    PtcParser/src/general_ptc_parser.cc
    PtcParser/src/ptc_1_0_parser.cc
    PtcParser/src/ptc_2_0_parser.cc
    PtcParser/src/ptc_parser.cc)
target_include_directories(HesaiPtcParser PUBLIC
    PtcParser/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/PtcParser/include>
    $<INSTALL_INTERFACE:include/PtcParser>
)
target_link_libraries(HesaiPtcParser PUBLIC HesaiLidar HesaiPtcClient)
set_target_properties(HesaiPtcParser PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
file(GLOB_RECURSE _ptc_parser_headers
    PtcParser/include/*.h
)

add_library(HesaiPtcClient
    PtcClient/src/ptc_client.cc
    PtcClient/src/tcp_client.cc
    PtcClient/src/tcp_ssl_client.cc
)
target_include_directories(HesaiPtcClient PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/PtcClient/include>
    $<INSTALL_INTERFACE:include/PtcClient>
)
target_link_libraries(HesaiPtcClient PUBLIC HesaiLidar HesaiPtcParser OpenSSL::SSL OpenSSL::Crypto)
set_target_properties(HesaiPtcClient PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
file(GLOB_RECURSE _ptc_client_headers
    PtcClient/include/*.h
)

add_library(HesaiSource
    Source/src/source.cc
    Source/src/socket_source.cc
    Source/src/pcap_source.cc
    Source/src/pcap_saver.cc
)
target_link_libraries(HesaiSource
    PUBLIC
    HesaiLidar
    HesaiPtcClient)
target_include_directories(HesaiSource PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Source/include>
    $<INSTALL_INTERFACE:include/Source>
)
set_target_properties(HesaiSource PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
file(GLOB_RECURSE _hesai_source_headers
    Source/include/*.h
)

add_library(HesaiLogging
    Logger/src/logger.cc)
target_include_directories(HesaiLogging PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Logger/include>
    $<INSTALL_INTERFACE:include/Logger>
)
target_link_libraries(HesaiLogging PUBLIC Threads::Threads)
set_target_properties(HesaiLogging PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
file(GLOB_RECURSE _logger_headers
    Logger/include/*.h
)

###########################################
# Install library archives
###########################################
install(
    TARGETS
    HesaiLidar
    HesaiSource
    HesaiLogging
    HesaiPtcClient
    HesaiPtcParser
)

###########################################
# Install header files
###########################################
install(
    FILES
    include/auto_tick_count.h
    include/plat_utils.h
    DESTINATION
    include
)

install(
    FILES
    ${_lidar_headers}
    DESTINATION include/Lidar
)

install(
    FILES
    ${_container_headers}
    DESTINATION include/Container
)

install(
    FILES
    ${_udpprotocol_headers}
    DESTINATION include/UdpProtocol
)

install(
    FILES
    ${_udpparser_headers}
    DESTINATION include/UdpParser
)

install(
    FILES
    ${_ptc_client_headers}
    DESTINATION include/PtcClient
)

install(
    FILES
    ${_ptc_parser_headers}
    DESTINATION include/PtcParser
)

install(
    FILES
    ${_hesai_source_headers}
    DESTINATION include/Source
)

install(
    FILES
    ${_logger_headers}
    DESTINATION include/Source
)
