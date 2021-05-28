load("@rules_cc//cc:defs.bzl", "cc_test")

# cc_library(
#     name = "src",
#     srcs = ["named_vector.cc",
#             "state.cc",
#             "measurement.cc",
#             "sensor.cc",
#             "dynamics_model.cc"],
#     hdrs = ["named_vector.h",
#             "state.h",
#             "measurement.h",
#             "sensor.h",
#             "dynamics_model.h"],
# )

cc_library(
    name = "src",
    srcs = ["named_vector.cc"],
    hdrs = ["named_vector.h"],
)

cc_test(
    name = "dynamics_model_test",
    size = "small",
    srcs = ["dynamics_model_test.cc",],
    deps = ["@com_google_googletest//:gtest_main",
            "//src:src"],
)

cc_test(
    name = "named_vector_test",
    size = "small",
    srcs = ["named_vector_test.cc",],
    deps = ["@com_google_googletest//:gtest_main",
            "//src:src"],
)

cc_test(
    name = "state_test",
    size = "small",
    srcs = ["state_test.cc",],
    deps = ["@com_google_googletest//:gtest_main",
            "//src:src"],
)

cc_test(
    name = "measurement_test",
    size = "small",
    srcs = ["measurement_test.cc",],
    deps = ["@com_google_googletest//:gtest_main",
            "//src:src"],
)

cc_test(
    name = "sensor_test",
    size = "small",
    srcs = ["sensor_test.cc",],
    deps = ["@com_google_googletest//:gtest_main",
            "//src:src"],
)
