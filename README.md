
# 📦 README — Ứng dụng thuật toán A* điều khiển robot trong kho hàng

## 🎯 Mục tiêu bài toán

Chương trình mô phỏng robot di chuyển trong kho hàng tự động được biểu diễn dưới dạng lưới 2D. Robot cần tìm đường đi từ điểm bắt đầu (**start**) đến điểm đích (**goal**) sao cho **tổng chi phí di chuyển là nhỏ nhất**.

Bài toán được giải bằng **thuật toán tìm kiếm A***.

---

## 🧱 Mô hình kho hàng

Kho hàng được biểu diễn bằng ma trận 2 chiều, trong đó mỗi ô có ý nghĩa:

| Giá trị | Ý nghĩa | Chi phí di chuyển |
| ------- | ------- | ----------------- |
| 0       | Ô trống | 1                 |
| 1       | Vật cản | Không đi được     |
| 2       | Bùn lầy | 3                 |
| 3       | Đá      | 5                 |

Robot chỉ được phép di chuyển theo **4 hướng**:

* Lên
* Xuống
* Trái
* Phải

---

## 🧠 Nguyên lý thuật toán A*

Thuật toán A* đánh giá mỗi ô dựa trên hàm chi phí:

[
f(n) = g(n) + h(n)
]

Trong đó:

* ( g(n) ): chi phí thực tế từ điểm bắt đầu đến vị trí hiện tại
* ( h(n) ): chi phí ước tính từ vị trí hiện tại đến đích (Manhattan distance)
* ( f(n) ): tổng chi phí ước tính

Thuật toán luôn chọn ô có giá trị ( f(n) ) nhỏ nhất để mở rộng.

---

## 🧮 Heuristic sử dụng

Do robot chỉ di chuyển theo 4 hướng nên sử dụng **khoảng cách Manhattan**:

[
h(n) = |x_1 - x_2| + |y_1 - y_2|
]

Heuristic này đảm bảo không vượt quá chi phí thực tế, phù hợp yêu cầu của A*.

---

## ⚙️ Cấu trúc chương trình

Chương trình gồm các phần chính:

* Khởi tạo cấu hình chi phí ô
* Các hàm hỗ trợ:

  * Tạo node
  * Tính heuristic
  * Lấy các ô lân cận hợp lệ
  * Tái tạo đường đi
* Cài đặt thuật toán A*
* Hiển thị đường đi bằng text và đồ họa
* Hàm `main` để chạy chương trình

---

## ▶️ Cách chạy chương trình

Cài đặt thư viện cần thiết:

```bash
pip install numpy matplotlib
```

Chạy chương trình:

```bash
python baitap.py
```

---

## ✅ Kết quả đạt được

Chương trình tìm được đường đi tối ưu từ start đến goal với:

* Số bước di chuyển nhỏ nhất theo chi phí
* Tránh vật cản
* Hạn chế đi qua bùn lầy và đá nếu không cần thiết
* Hiển thị trực quan đường đi trên lưới

---

## 📌 Kết luận

Việc áp dụng thuật toán A* giúp robot tìm được đường đi tối ưu trong môi trường có nhiều loại địa hình khác nhau. Heuristic Manhattan kết hợp với chi phí từng loại ô giúp thuật toán hoạt động hiệu quả và chính xác.

Bài thực hành cho thấy khả năng ứng dụng mạnh mẽ của A* trong các bài toán tìm đường thực tế như robot tự hành trong kho hàng.
