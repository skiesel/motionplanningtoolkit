namespace math {
	void multiply(const std::vector<double> &m1, const std::vector<double> &m2, std::vector<double> &out) {
		std::vector<double> temp(16);
		for(unsigned int row = 0; row < 4; ++row) {
			for(unsigned int col = 0; col < 4; ++col) {
				double sum = 0;
				for(unsigned int i = 0; i < 4; i++) {
					double elem1 = m1[row * 4 + i];
					double elem2 = m2[col + 4 * i];
					sum += elem1 * elem2;
				}
				temp[row * 4 + col] = sum;
			}
		}
		for(unsigned int i = 0; i < 16; i++) out[i] = temp[i];
	}
}