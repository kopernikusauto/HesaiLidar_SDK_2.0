# ----------------------------------------#
# Hesai Tech <http://www.hesaitech.com>  #
# ----------------------------------------#

add_library(lidar_lib
    src/plat_utils.cc
    src/auto_tick_count.cc
    Container/src/ring_2d_shared.cc
    Container/src/ring_2dex.cc
)
target_include_directories(lidar_lib PUBLIC include Lidar UdpProtocol UdpParser/include UdpParser/src Container/include Container/src)
set_target_properties(lidar_lib PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)

if(WITH_PTCPARSER)
    add_library(ptcParser_lib
        PtcParser/src/general_ptc_parser.cc
        PtcParser/src/ptc_1_0_parser.cc
        PtcParser/src/ptc_2_0_parser.cc
        PtcParser/src/ptc_parser.cc)
    target_include_directories(ptcParser_lib PUBLIC PtcParser/include)
    target_link_libraries(ptcParser_lib PUBLIC lidar_lib ptcClient_lib)
    set_target_properties(ptcParser_lib PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
endif(WITH_PTCPARSER)

# Make each module an 'OBJECT' library, can be refered to by get_link_libraries
if(WITH_PTCCLIENT)
    add_library(ptcClient_lib
        PtcClient/src/ptc_client.cc
        PtcClient/src/tcp_client.cc
        PtcClient/src/tcp_ssl_client.cc
        )
    target_include_directories(ptcClient_lib PUBLIC PtcClient/include)
    target_link_libraries(ptcClient_lib PUBLIC lidar_lib ptcParser_lib OpenSSL::SSL OpenSSL::Crypto)
    set_target_properties(ptcClient_lib PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
endif(WITH_PTCCLIENT)

if(WITH_LIDARBOARD)
    add_library(source_lib
        Source/src/source.cc
        Source/src/socket_source.cc
        Source/src/pcap_source.cc
        Source/src/pcap_saver.cc
        )
    target_link_libraries(source_lib
        PUBLIC
        lidar_lib
        ptcClient_lib)
    target_include_directories(source_lib PUBLIC Source/include)
    set_target_properties(source_lib PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
endif(WITH_LIDARBOARD)

if(WITH_LOG)
    find_package(Threads)
    add_library(log_lib EXCLUDE_FROM_ALL
        Logger/src/logger.cc)
    target_link_libraries(log_lib PUBLIC Threads::Threads)
    set_target_properties(log_lib PROPERTIES CXX_STANDARD_REQUIRED TRUE CXX_STANDARD 17)
endif(WITH_LOG)
