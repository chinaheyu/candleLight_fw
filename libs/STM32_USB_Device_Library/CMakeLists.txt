project(STM32_USB_Device_Library)

set(SOURCES
    Core/Inc/usbd_def.h
    Core/Inc/usbd_ctlreq.h
    Core/Src/usbd_ctlreq.c
    Core/Inc/usbd_ioreq.h
    Core/Src/usbd_ioreq.c
    Core/Inc/usbd_core.h
    Core/Src/usbd_core.c
)

set(INCLUDE_DIRS
    Core/Inc/
    config/
)

add_library(STM32_USB_Device_Library_STM32F042x6 STATIC ${SOURCES})
    target_include_directories(STM32_USB_Device_Library_STM32F042x6 PUBLIC ${INCLUDE_DIRS})
    target_compile_options(STM32_USB_Device_Library_STM32F042x6 PRIVATE ${CPUFLAGS_F0} -Wno-unused-parameter -DSTM32F0)
    target_link_libraries(STM32_USB_Device_Library_STM32F042x6 PRIVATE STM32_HAL_STM32F042x6)

add_library(STM32_USB_Device_Library_STM32F072xB STATIC ${SOURCES})
    target_include_directories(STM32_USB_Device_Library_STM32F072xB PUBLIC ${INCLUDE_DIRS})
    target_compile_options(STM32_USB_Device_Library_STM32F072xB PRIVATE ${CPUFLAGS_F0} -Wno-unused-parameter -DSTM32F0)
    target_link_libraries(STM32_USB_Device_Library_STM32F072xB PRIVATE STM32_HAL_STM32F072xB)

add_library(STM32_USB_Device_Library_STM32F407xE STATIC ${SOURCES})
    target_include_directories(STM32_USB_Device_Library_STM32F407xE PUBLIC ${INCLUDE_DIRS})
    target_compile_options(STM32_USB_Device_Library_STM32F407xE PRIVATE ${CPUFLAGS_F4} -Wno-unused-parameter -DSTM32F4)
    target_link_libraries(STM32_USB_Device_Library_STM32F407xE PRIVATE STM32_HAL_STM32F407xE)

add_library(STM32_USB_Device_Library_STM32G0B1xK STATIC ${SOURCES})
    target_include_directories(STM32_USB_Device_Library_STM32G0B1xK PUBLIC ${INCLUDE_DIRS})
    target_compile_options(STM32_USB_Device_Library_STM32G0B1xK PRIVATE ${CPUFLAGS_G0} -Wno-unused-parameter -DSTM32G0)
    target_link_libraries(STM32_USB_Device_Library_STM32G0B1xK PRIVATE STM32_HAL_STM32G0B1xK)

add_library(STM32_USB_Device_Library_STM32G431x8 STATIC ${SOURCES})
    target_include_directories(STM32_USB_Device_Library_STM32G431x8 PUBLIC ${INCLUDE_DIRS})
    target_compile_options(STM32_USB_Device_Library_STM32G431x8 PRIVATE ${CPUFLAGS_G4} -Wno-unused-parameter -DSTM32G4)
    target_link_libraries(STM32_USB_Device_Library_STM32G431x8 PRIVATE STM32_HAL_STM32G431x8)

add_library(STM32_USB_Device_Library_SCUT_candleLightFD STATIC ${SOURCES})
    target_include_directories(STM32_USB_Device_Library_SCUT_candleLightFD PUBLIC ${INCLUDE_DIRS})
    target_compile_options(STM32_USB_Device_Library_SCUT_candleLightFD PRIVATE ${CPUFLAGS_G4} -Wno-unused-parameter -DSTM32G4)
    target_link_libraries(STM32_USB_Device_Library_SCUT_candleLightFD PRIVATE STM32_HAL_SCUT_candleLightFD)
