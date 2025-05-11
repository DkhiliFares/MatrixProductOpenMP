#include <rclcpp/rclcpp.hpp>
#include <random>
#include "matrix_multiply_service/srv/matrix_multiply.hpp"

using MatrixMultiply = matrix_multiply_service::srv::MatrixMultiply;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("matrix_multiply_client");
  auto client = node->create_client<MatrixMultiply>("matrix_multiply");

  // Attendre que le service soit disponible
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for service...");
  }

  // Créer une requête avec deux matrices 100x100
  auto request = std::make_shared<MatrixMultiply::Request>();
  request->rows_a = 100;
  request->cols_a = 100;
  request->rows_b = 100;
  request->cols_b = 100;

  // Générer des matrices avec des valeurs aléatoires
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 10.0); // Valeurs entre 0 et 10

  request->matrix_a.resize(100 * 100);
  request->matrix_b.resize(100 * 100);
  for (int i = 0; i < 100 * 100; ++i) {
    request->matrix_a[i] = dis(gen);
    request->matrix_b[i] = dis(gen);
  }

  // Mesurer le temps d'exécution
  auto start = std::chrono::high_resolution_clock::now();

  // Envoyer la requête
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result.get();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    RCLCPP_INFO(node->get_logger(), "Result matrix (%dx%d) received in %ld ms",
                response->rows_c, response->cols_c, duration);
    // Afficher un échantillon du résultat (premiers éléments pour éviter une sortie trop longue)
    RCLCPP_INFO(node->get_logger(), "Sample of result (first 5 elements):");
    for (int i = 0; i < std::min(5, (int)response->result.size()); ++i) {
      std::cout << response->result[i] << " ";
    }
    std::cout << std::endl;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
