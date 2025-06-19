#include <iostream>
#include <string>

uint8_t calcularChecksum(const std::string& str) {
    uint8_t checksum = 0;
    for (char c : str) {
        checksum += c;
    }
    return checksum;
}

int main() {
    std::string usuario;
    std::string contrasena;

    std::cout << "Por favor, introduce tu nombre de usuario: ";
    std::cin >> usuario;

    std::cout << "Por favor, introduce tu contraseña: ";
    std::cin >> contrasena;

    uint8_t checksumUsuario = calcularChecksum(usuario);
    uint8_t checksumContrasena = calcularChecksum(contrasena);

    std::cout << "Checksum del usuario: " << static_cast<int>(checksumUsuario) << "\n";
    std::cout << "Checksum de la contraseña: " << static_cast<int>(checksumContrasena) << "\n";

    std::cout << "ok\n";

    return 0;
}
