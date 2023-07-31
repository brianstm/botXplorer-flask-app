import numpy as np


def calculate_confidence_level(covariance_data):
    covariance_data = np.reshape(covariance_data, (6, 6))
    covariance_position = np.array(covariance_data[:2, :2])
    covariance_orientation = np.array(covariance_data[5:6, 5:6])
    eigenvalues_position, _ = np.linalg.eig(covariance_position)
    eigenvalues_orientation, _ = np.linalg.eig(covariance_orientation)
    total_uncertainty = np.sum(eigenvalues_position) + \
        np.sum(eigenvalues_orientation)
    confidence_level = max(0, 100 - total_uncertainty * 100)

    return confidence_level


covariance_data = [0.010835141962386068, 0.019986527241657174, 0.0, 0.0, 0.0, 0.0, 0.01998652724165717, 0.042168224557572506, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.010001557539512205]

confidence_level = calculate_confidence_level(covariance_data)
print(f"Confidence Level: {confidence_level:.2f}%")
