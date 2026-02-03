if(CONFIG_ZMK_BEHAVIOR_REFCOUNT_KEY)
  zephyr_library()
  zephyr_library_sources(
    src/behavior_refcount_key.c
  )
endif()
