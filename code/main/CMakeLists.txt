idf_component_register(SRCS "mecoen_main.cpp" "server.cpp" #[["mecoen_adc.cpp"]] #[["mecoen_time.cpp"]]
						"mecoen_fft.cpp" "mecoen_i2c.cpp" "mecoen_storage.cpp" "mecoen_commons.cpp"
						"mecoen_wifi.cpp"
                    INCLUDE_DIRS ".")
