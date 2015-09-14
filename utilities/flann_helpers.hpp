namespace flann_helpers {

	struct Distance {
		typedef bool is_kdtree_distance;

		typedef double ElementType;
		typedef typename flann::Accumulator<double>::Type ResultType;


		Distance(const std::vector<bool> &rotationalCoordinate) : rotationalCoordinate(rotationalCoordinate) {}

		template <typename Iterator1, typename Iterator2>
		ResultType operator()(Iterator1 a, Iterator2 b, size_t size, ResultType /*worst_dist*/ = -1) const {
			ResultType result = ResultType();
			ResultType diff;
			for(size_t i = 0; i < size; ++i ) {
				if(rotationalCoordinate[i]) {
					diff = *a++ - *b++;
					diff += (diff > M_PI) ? -2*M_PI : (diff < -M_PI) ? 2*M_PI : 0;
				} else {
					diff = *a++ - *b++;
				}
				result += diff*diff;
			}
			return result;
		}

		template <typename U, typename V>
		inline ResultType accum_dist(const U& a, const V& b, int index) const {
			if(rotationalCoordinate[index]) {
				ResultType diff;
				diff = a - b;
				diff += (diff > M_PI) ? -2 * M_PI : (diff < -M_PI) ? 2 * M_PI : 0;
				return diff * diff;
			}

			return (a-b)*(a-b);
		}

		std::vector<bool> rotationalCoordinate;
	};

};