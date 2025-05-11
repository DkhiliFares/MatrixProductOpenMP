#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <omp.h>
#include "matrix_multiply_service/srv/matrix_multiply.hpp"

using MatrixMultiply = matrix_multiply_service::srv::MatrixMultiply;

class MatrixMultiplyServer : public rclcpp::Node {
public:
  MatrixMultiplyServer() : Node("matrix_multiply_server") {
    service_ = create_service<MatrixMultiply>(
      "matrix_multiply", 
      std::bind(&MatrixMultiplyServer::handle_service, this, 
                std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Matrix Multiply Service Ready");
  }

private:
  void handle_service(
    const std::shared_ptr<MatrixMultiply::Request> request,
    std::shared_ptr<MatrixMultiply::Response> response) {
    // Vérification des dimensions
    if (request->rows_a * request->cols_a != request->matrix_a.size() ||
        request->rows_b * request->cols_b != request->matrix_b.size() ||
        request->cols_a != request->rows_b) {
      RCLCPP_ERROR(this->get_logger(), "Invalid matrix dimensions");
      return;
    }

    // Allocation de la matrice résultat
    response->rows_c = request->rows_a;
    response->cols_c = request->cols_b;
    response->result.resize(response->rows_c * response->cols_c);

    // Multiplication matricielle avec OpenMP
    #pragma omp parallel for collapse(2)
    for (int i = 0; i < request->rows_a; ++i) {
      for (int j = 0; j < request->cols_b; ++j) {
        double sum = 0.0;
        for (int k = 0; k < request->cols_a; ++k) {
          sum += request->matrix_a[i * request->cols_a + k] *
                 request->matrix_b[k * request->cols_b + j];
        }
        response->result[i * response->cols_c + j] = sum;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Matrix multiplication completed");
  }

  rclcpp::Service<MatrixMultiply>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MatrixMultiplyServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
