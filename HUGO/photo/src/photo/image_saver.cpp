#include "image_saver.hpp"
#include <ctime> // Para std::strftime

ImageSaver::ImageSaver()
    : Node("image_saver"), image_saved_(false) {
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",
        10,
        std::bind(&ImageSaver::imageCallback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Nodo de captura de imagen inicializado.");
}

ImageSaver::~ImageSaver() {}

void ImageSaver::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!image_saved_) { // Verifica si ya se guardó una imagen
        try {
            // Convierte la imagen ROS a un formato OpenCV
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

            // Genera un nombre único para la imagen usando un timestamp
            std::string filename = "/home/usuario/imagenes/destino_" + getTimestamp() + ".jpg";

            // Guarda la imagen en el sistema de archivos
            cv::imwrite(filename, cv_ptr->image);

            // Informa que la imagen se guardó con éxito
            RCLCPP_INFO(this->get_logger(), "Imagen guardada en: %s", filename.c_str());

            // Marca que ya se guardó una imagen, evitando capturas adicionales
            image_saved_ = true;
        } catch (cv_bridge::Exception &e) {
            // Maneja errores en la conversión de la imagen
            RCLCPP_ERROR(this->get_logger(), "Error al procesar la imagen: %s", e.what());
        }
    }
}

std::string ImageSaver::getTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    // Define un buffer para almacenar el formato de tiempo
    char buffer[20]; // Suficiente para "YYYYMMDD_HHMMSS\0"
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&now_c));

    return std::string(buffer); // Retorna la cadena formateada
}

