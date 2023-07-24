# Copyright (c) 2023 Laird Connectivity.
# SPDX-License-Identifier: LicenseRef-LairdConnectivity-Clause

if(CONFIG_LCZ_FEM)
    zephyr_library()
    zephyr_library_sources(fem_gpio.c)

    if(CONFIG_LCZ_FEM_DEBUG)
        zephyr_library_sources(fem_debug.c)
    endif()

    if(CONFIG_BT OR CONFIG_NRF_802154_SL)

        if(CONFIG_LCZ_FEM_INTERNAL_ANTENNA)
            set(PT_TYPE "internal")
        else()
            set(PT_TYPE "external")
        endif()

        if(CONFIG_LCZ_FEM_REGION EQUAL 0)
            set(PT_REGION "fcc_ic")
        elseif(CONFIG_LCZ_FEM_REGION EQUAL 1)
            set(PT_REGION "rcm")
        elseif(CONFIG_LCZ_FEM_REGION EQUAL 2)
            set(PT_REGION "ce")
        else()
            set(PT_REGION "test")
            message("Test region not valid for production")
        endif()

        if(CONFIG_MPSL_FEM_POWER_MODEL_NRF21540_LAIRD_CONNECTIVITY)
            if(CONFIG_LCZ_FEM_REGION EQUAL 2)
                zephyr_library_sources(fem_power_model_ce.c)
            else() # Errata Model
                zephyr_library_sources(fem_power_model.c)
            endif()
        endif() # Power Model

        if(NOT CONFIG_LCZ_FEM_REGION EQUAL 2)
            zephyr_library_sources(fem_power.c)
            zephyr_library_include_directories(power_table)

            #
            # Generate power table file name based on type of antenna and region
            #

            # BL5340PA is currently the only option.
            set(PT_PATH "power_table/bl5340pa")

            zephyr_library_sources(${PT_PATH}/${PT_REGION}_${PT_TYPE}.c)
        endif() # Power Table

    endif() # BT or 802.15.4
endif() # FEM_DRIVER
