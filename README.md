# polygons
разные методы для работы с полигонами
# вспомагательные типы
`vec2`, `vec2u`, `vec3` - типы, аналогичные типам glsl, `s32`, `u8` и т.п. - `signed 32 bit integer`, `unsigned 8 bit integer` и т.д.
# Poly
Класс полигона: представляет из себя вектор точек в двумерном пространстве,упорядоченых в порядке обхода полигона по часовой стрелке.
Если порядок обратный то считается, что полигон вывернут. Пока что не гарантирует отсутствие самопересечений и повторов вершин.
Для него перегружен `operator[]`, чтобы можно было обращаться у вершинам по модулю
# Площадь и центр масс
Методы `Poly::Area` и `Poly::MassCenter`, `Poly::AreaXCenter` считают, соответвенно площадь(массу), центр массы (при условии равномерного распределения её по площади) и их произведение соответственно, работают за **O(N)**, где N - размер полигона.
# Принадлежность точки внутренной части полигона
Этот метод работает за **O(N) + O(Q * log(N/Q) * M)** N - размер полигона, Q - число монотонных участков, M - число проверяемых точек.
Метод `MonotonicZone Poly::DivideToMonotonics()` делит полигон на монотонные участки (пары индексов + тип участка), а `Poly::IsInsideInt(const MonotonicZone&, vec2)` возвращает 1, если точка внутри полигона и 0, если снаружи. Если полигон вывернут, то возвращает -1 внутри. Если полигон имеет петли, то взависимости от их направления учитывает их вклад.
# Минимальная выпуклая оболочка
`std::vector<u32> MinimalHull(const PointCloud& cloud)` выделяет индексы вершин принадлежащих выпуклой оболочке cloud по часовой стрелке. Скорость - **O(N ^ 3/2)** в среднем, где N - число вершин, планируется ускорить до **O(n * logN)** 
