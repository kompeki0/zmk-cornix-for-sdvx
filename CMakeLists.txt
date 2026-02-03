if(CONFIG_ZMK_BEHAVIOR_REFCOUNT_KEY)
  zephyr_library()
  zephyr_include_directories(include)
  zephyr_library_sources(
    src/behavior_refcount_key.c
  )
endif()
