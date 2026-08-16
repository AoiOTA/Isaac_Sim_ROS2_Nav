from robot_bringup.amcl_clear_localization_buffer import (
    AmclClearLocalizationBuffer,
)


def test_clear_localization_buffer_is_a_no_op_echo():
    response = object()

    assert AmclClearLocalizationBuffer._clear_localization_buffer(
        None, response) is response
