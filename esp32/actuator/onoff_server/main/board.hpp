#ifndef BOARD_H
#define BOARD_H

#include <cstdint>

/*
 * Lớp trừu tượng cho phần "xuất tín hiệu actuator ra phần cứng thật".
 * Hiện chưa có relay/đèn thật nên dùng làm chỉ báo tạm:
 *   - ESP32 (gốc)      : LED GPIO thường, sáng = ON, tắt = OFF
 *   - ESP32-H2 / ESP32-C6 : LED RGB địa chỉ hoá (WS2812) tích hợp sẵn trên
 *     devkit (2 dòng chip này không có LED GPIO rời như ESP32 gốc) - xanh lá
 *     = ON, tắt hẳn = OFF, chớp xanh dương vài lần khi có setpoint mới.
 *
 * Khi có relay/đèn thật, chỉ cần sửa nhánh CONFIG_IDF_TARGET_ESP32 trong
 * board.cpp (đổi gpio_set_level thành điều khiển relay thật) - không cần đụng
 * tới actuator_main.cpp, vì file đó chỉ gọi qua API dưới đây.
 */

// Gọi 1 lần trong app_main() trước khi dùng 2 hàm bên dưới. Tạo sẵn 1 task
// nền xử lý hiệu ứng nhấp nháy, không chặn (block) người gọi.
void board_actuator_init(void);

// Gọi khi actuator đổi trạng thái ON/OFF thật. Task nền tự chuyển sang màu/
// trạng thái ổn định tương ứng.
void board_actuator_notify_onoff(uint8_t onoff);

// Gọi khi nhận setpoint mới (khác giá trị hiện tại). Task nền chớp nhanh vài
// lần để báo "vừa nhận lệnh", rồi quay lại đúng trạng thái ON/OFF hiện tại.
void board_actuator_notify_setpoint(uint16_t setpoint);

#endif