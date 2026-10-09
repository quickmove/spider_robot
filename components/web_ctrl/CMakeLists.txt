idf_component_register(
    SRCS "src/web.c" "src/ws_cmd.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "private_include"
    REQUIRES
        motion
        posture
        servo
        gait
        kinematics
        log
        json
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
)

# 控制页 gzip 后内嵌：22.8KB -> ~7KB。Flash 余量紧张（WiFi+httpd 已占 84%），
# 而 httpd 侧只要发一个 Content-Encoding: gzip，浏览器会自动解压。
# IDF 的 EMBED_FILES 不支持自动压缩，故在配置期用 Python 压一次。
set(PAGE_SRC "${CMAKE_CURRENT_LIST_DIR}/index.html")
set(PAGE_GZ  "${CMAKE_CURRENT_BINARY_DIR}/index_html.gz")

add_custom_command(
    OUTPUT  "${PAGE_GZ}"
    COMMAND ${PYTHON} -c
        "import gzip,sys;d=open(sys.argv[1],'rb').read();g=gzip.GzipFile(sys.argv[2],'wb',9,mtime=0);g.write(d);g.close()"
        "${PAGE_SRC}" "${PAGE_GZ}"
    DEPENDS "${PAGE_SRC}"
    COMMENT "gzip 控制页"
    VERBATIM
)
add_custom_target(spider_page_gz ALL DEPENDS "${PAGE_GZ}")
add_dependencies(${COMPONENT_LIB} spider_page_gz)

target_sources(${COMPONENT_LIB} PRIVATE "${PAGE_GZ}")
target_add_binary_data(${COMPONENT_LIB} "${PAGE_GZ}" BINARY)
