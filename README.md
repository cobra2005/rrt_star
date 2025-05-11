# # Báo cáo: Chương trình RRT\* Path Planning với Animation

**Người thực hiện**: Đào Hoàng Thái
**Ngày**: 8/5/2025

## 1. Giới thiệu

Chương trình này triển khai thuật toán RRT\* (Rapidly-exploring Random Tree Star) để tìm đường đi tối ưu trong môi trường 2D có chướng ngại vật, với giao diện trực quan bằng Pygame.

## 2. Mục tiêu

-   Triển khai thuật toán RRT\* để tìm đường đi từ điểm bắt đầu đến điểm đích
-   Trực quan hóa quá trình tìm kiếm đường đi
-   Cung cấp giao diện tương tác để điều khiển quá trình mô phỏng

## 3. Kiến trúc chương trình

### 3.1. Các lớp chính

-   **Node**: Đại diện cho một nút trong cây RRT\*

    -   Thuộc tính: tọa độ (x,y), chi phí (cost), nút cha (parent)

-   **RRTStar**: Triển khai thuật toán RRT\*

    -   Các phương thức chính:

        -   `step()`: Thực hiện một bước của thuật toán
        -   `sample()`: Lấy mẫu ngẫu nhiên
        -   `nearest()`: Tìm nút gần nhất
        -   `steer()`: Tạo nút mới từ nút hiện tại
        -   `obstacle_free()`: Kiểm tra đường đi không va chạm

### 3.2. Thông số cấu hình

- max_iter = 5000 # Số lần lặp tối đa
- step_size = 25 # Khoảng cách bước tối đa
- goal_sample_rate = 0.1 # Xác suất lấy mẫu về đích
- gamma = 150.0 # Tham số điều chỉnh bán kính tìm kiếm

## 4. Thuật toán RRT\*

### 4.1. Các bước chính

1.  Khởi tạo cây với nút bắt đầu
2.  Lặp lại cho đến khi đạt max_iter:

    -   Lấy mẫu ngẫu nhiên trong không gian
    -   Tìm nút gần nhất trong cây
    -   Tạo nút mới theo hướng mẫu ngẫu nhiên
    -   Kiểm tra va chạm với chướng ngại vật
    -   Tìm các nút lân cận để tối ưu hóa đường đi
    -   Thêm nút mới vào cây
    -   Rewire các nút lân cận nếu tìm được đường đi tốt hơn

3.  Trả về đường đi tối ưu nhất sau khi hoàn thành tất cả lần lặp

### 4.2. Đặc điểm nổi bật

-   Tìm đường đi tối ưu về chi phí (độ dài đường đi)
-   Có cơ chế rewire để liên tục cải thiện đường đi
-   Chạy đủ số lần lặp trước khi trả về kết quả cuối cùng

## 5. Giao diện người dùng

### 5.1. Tính năng tương tác

-   SPACE: Tạm dừng/Tiếp tục

-   T: Bật/tắt hiển thị cây

-   A: Bật/tắt animation

-   UP/DOWN: Điều chỉnh tốc độ animation

-   R: Reset chương trình

### 5.2. Thông tin hiển thị

-   Số lần lặp hiện tại/tổng số
-   Số nút trong cây
-   Số đường đi tìm được tới đích
-   Tốc độ animation
-   Độ dài đường đi (nếu tìm thấy)

## 6. Kết quả đạt được

-   Hiển thị trực quan quá trình xây dựng cây RRT\*
-   Đường đi tối ưu được hiển thị bằng màu đỏ
-   Có thể quan sát quá trình tối ưu hóa đường đi theo thời gian thực
-   Hỗ trợ tiếng Việt trong giao diện

## 7. Hạn chế và hướng phát triển

### 7.1. Hạn chế

-   Chưa hỗ trợ đa luồng, có thể bị giật khi chạy với số lần lặp lớn
-   Chưa có chức năng lưu/load cấu hình môi trường
-   Chưa tối ưu hiệu năng cho môi trường phức tạp

### 7.2. Hướng phát triển

-   Thêm nhiều loại chướng ngại vật khác nhau
-   Triển khai phiên bản đa luồng để cải thiện hiệu năng
-   Thêm chức năng lưu/load bản đồ
-   Phát triển thành thư viện có thể tái sử dụng

## 8. Kết luận

Chương trình đã triển khai thành công thuật toán RRT\* với giao diện trực quan bằng Pygame, cho phép quan sát quá trình tìm kiếm đường đi tối ưu trong môi trường 2D có chướng ngại vật. Đây là công cụ hữu ích để nghiên cứu và giảng dạy về các thuật toán lập kế hoạch đường đi.
