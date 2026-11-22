# 遥测帧契约共享组件（纯 C，无 IDF 依赖）。
# 通过各工程的 EXTRA_COMPONENT_DIRS 引用，契约一处维护。
idf_component_register(
    SRCS "telem_frame.c"
    INCLUDE_DIRS "include"
)
