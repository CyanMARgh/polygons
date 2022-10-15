#include "buffer.h"
#include "polygon.h"
#include "primitives.h"
#include "utils.h"
#include <queue>

Reindexed_Cloud geom::to_sorted_vertical(const Point_Cloud& cloud) {
	const Point_Cloud* t = &cloud;
	u32 n = cloud.size();
	Reindexed_Cloud rc(std::vector<u32>(n), t);
	for(u32 i = 0; i < n; i++) rc[i] = i;
	std::sort(rc.begin(), rc.end(), [t](u32 a, u32 b) { 
		vec2 pa = t->at(a), pb = t->at(b);
		return pa.y == pb.y ? pa.x > pb.x : pa.y > pb.y; 
	});	
	return rc;
}

std::pair<float, vec2> geom::bis_inter(Line X, Line Y, Line Z) {
	vec2 a = X.a, b = X.b, c = Y.a, d = Y.b, e = Z.a, f = Z.b;
	vec2 n1 = rrot(normalize(b - a)), n2 = rrot(normalize(d - c)), n3 = rrot(normalize(f - e));

	Mat2x2 M = Mat2x2::from_rows(n1 - n2, n2 - n3);
	vec2 V = {dot(a, n1) - dot(c, n2), dot(c, n2) - dot(e, n3)};

	vec2 Q = M.inv() * V;
	float h = dot(Q - a, n1);

	return {h, Q};
}

bool geom::sk_event::operator<(const geom::sk_event& e2) const {
	return h > e2.h;
}
bool geom::sk_event_2::operator<(const geom::sk_event_2& e2) const {
	return h > e2.h;
}

geom::skeleton geom::make_skeleton_from_convex(const Poly& P) {
	u32 n = P.size;
	struct edge {
		u32 id0;	// id of (parallel) origin segment
		u32 left_edge, right_edge;
		u32 left_vert, right_vert;

		bool is_isolated() const {
			return right_edge == left_edge;
		}
		static void collapse(edge* edge_list, u32 edge_id, u32 new_vertice) {
			edge *B = edge_list + edge_id;
			edge *A = edge_list + B->left_edge, *C = edge_list + B->right_edge;
			A->right_edge = B->right_edge, C->left_edge = B->left_edge;
			A->right_vert = C->left_vert = new_vertice;
			B->left_edge = B->right_edge = edge_id;
		}
	};
	auto is_final = [](sk_event e, const edge* edge_list) -> bool { 
		return edge_list[e.edge_C].right_edge == e.edge_A && e.edge_A != e.edge_B && e.edge_B != e.edge_C; 
		//return false;
	};

	std::vector<edge> edge_list(n);
	for(u32 i = 0; i < n; i++) {
		u32 in = (i + 1) % n, ip = (i + n - 1) % n;
		edge_list[i] = {i, ip, in, i, in};
	}
	auto line_at_vi = [&P] (u32 id0) -> Line {
		return {P[id0], P[id0 + 1]};
	};
	// auto line_at = [&P] (edge e) -> Line {
	// 	return {P[e.id0], P[e.id0 + 1]};
	// };
	auto line_at = [&P] (const edge* edge_list, u32 edge_id) -> Line {
		u32 id0 = edge_list[edge_id].id0;
		return {P[id0], P[id0 + 1]};
	};
	auto make_event = [&line_at] (const edge* edge_list, u32 edge_id) -> sk_event {
		edge B = edge_list[edge_id];
		u32 edge_A = B.left_edge, edge_C = B.right_edge;
		auto hQ = bis_inter(line_at(edge_list, edge_A), line_at(edge_list, edge_id), line_at(edge_list, edge_C));
		return {hQ.first, hQ.second, edge_A, edge_id, edge_C};
	};
	auto is_inside_bis_triangle = [](vec2 A, vec2 B, vec2 C, vec2 D, vec2 Q) -> bool {
		vec2 ba = normalize(A - B), cb = normalize(B - C), cd = normalize(D - C), bq = Q - B, cq = Q - C;
		return dot(ba + cb, bq) < 0.f && dot(cb - cd, cq) > 0.f; 
	};
	auto check_split_event = [&P, &is_inside_bis_triangle](const edge* edge_list, sk_event e) -> bool {
		edge e_B = edge_list[e.edge_B];
		edge e_Bl = edge_list[e_B.left_edge], e_Br = edge_list[e_B.right_edge];

		u32 iv_A = e_Bl.left_vert, iv_B = e_Bl.right_vert, iv_C = e_Br.left_vert, iv_D = e_Br.right_edge;
		return is_inside_bis_triangle(P[iv_A], P[iv_B], P[iv_C], P[iv_D], e.Q);
	};
	auto is_valid = [&check_split_event](sk_event e, const edge* edge_list) -> bool {
		if(e.type != sk_event::SPLIT) {
			edge e_B = edge_list[e.edge_B];
			return e_B.right_edge == e.edge_C && e_B.left_edge == e.edge_A &&
			!(edge_list[e.edge_B].is_isolated() || edge_list[e.edge_A].is_isolated() || edge_list[e.edge_C].is_isolated());
		} else {
			return check_split_event(edge_list, e);
		}
	};
	//auto fix_split_event = [](..., sk_event e) -> std::pair<sk_event, sk_event> { ... }; 

	// struct sk_event {
	// 	float h;
	// 	vec2 Q;
	// 	u32 edge_A, edge_B, edge_C;
	// 	u32 left_vert, right_vert, third_vert;

	// 	enum type_t { EDGE, SPLIT, END } type = EDGE;
	// };

	auto make_split_event = [&line_at_vi, n] (/*edge* edge_list, */u32 o0, u32 k0) -> sk_event {
		Line AB = line_at_vi(k0), BC = line_at_vi(k0 + 1), DE = line_at_vi(o0);

		printf("AB: (%f %f) (%f %f)\n", AB.a.x, AB.a.y, AB.b.x, AB.b.y);
		printf("BC: (%f %f) (%f %f)\n", BC.a.x, BC.a.y, BC.b.x, BC.b.y);
		printf("DE: (%f %f) (%f %f)\n", DE.a.x, DE.a.y, DE.b.x, DE.b.y);

		auto hQ = bis_inter(AB, BC, DE);
		printf("h: %f\n", hQ.first);
		printf("(4) %u %u %u\n", k0, o0, (k0 + 1) % n);
		sk_event se = {hQ.first, hQ.second, k0, o0, (k0 + 1) % n};
		se.type = sk_event::SPLIT;
		se.third_vert = (k0 + 1) % n;
		return se;
		// if(is_inside_bis_triangle(P[o0], P[o0 + 1], P[o0 + 2], P[o0 + 3], hQ.second)) { // check is valid later
		// 	return {true, hQ.first, hQ.second, };
		// } else {
		// 	return {false, {}};
		// }
		// u32 edge_knife_2 = edge_list[edge_knife].right_edge;
		// u32 slicing_edge_r = edge_list[slicing_edge].right_edge;
		// u32 slicing_edge_l = edge_list[slicing_edge].left_edge;
	};
	std::priority_queue<sk_event> events;
	auto add_all_split_events = [&events, &P, &make_split_event, n] () -> void {
		if(n < 5) return;
		for(u32 k0 = 0; k0 < n; k0++) {
			if(cross(P[k0 + 1] - P[k0], P[k0 + 2] - P[k0 + 1]) <= 0) continue;
			for(u32 o0 = (k0 + 3) % n; o0 != (k0 + n - 1) % n; o0 = (o0 + 1) % n) {
				events.push(make_split_event(o0, k0));
				printf("(3)\n");
			} 
		}
	};

	skeleton S;

	auto commit_event = [&S, &is_final] (const edge* edge_list, sk_event e) {
		edge B = edge_list[e.edge_B];
		sk_event se = {e.h, e.Q, e.edge_A, e.edge_B, e.edge_C, B.left_vert, B.right_vert, e.third_vert};
		se.type = e.type;
		if(is_final(e, edge_list)) {
			printf("final!\n");
			se.type = sk_event::END;
			se.third_vert = edge_list[e.edge_C].right_vert;
		}
		S.push_back(se);
	};
	auto commit_split_event = [&S] (const edge* edge_list, sk_event e) {
		//
	};

	for(u32 i = 0; i < n; i++) {
		events.push(make_event(&(edge_list[0]), i));
	}
	add_all_split_events();
	// struct sk_event {
	// 	float h;
	// 	vec2 Q;
	//(in split) 
	// 	u32 edge_A, edge_B, edge_C;
	//  knfe_l     splitted   knife_r 
	// 	u32 left_vert, right_vert, third_vert;

	// 	enum type_t { EDGE, SPLIT, END } type = EDGE;
	// };
	
	while(!events.empty()) {
		sk_event e = events.top();
		events.pop();
		if(e.h < 0 || !is_valid(e, &(edge_list[0]))) continue;
		if(e.type == sk_event::SPLIT) {
			// WRONG H CALCULATING FOR SPLIT EVENTS (PROBABLY CAUSE OF WRONG INDEXES)
			printf("(2)\n");
			//if(!check_split_event(&(edge_list[0]), e)) continue;

			// u32 id0;	// id of (parallel) origin segment
			// u32 left_edge, right_edge;
			// u32 left_vert, right_vert;
			// add reconections
			edge *e_A = &(edge_list[e.edge_A]);
			edge *e_B = &(edge_list[e.edge_B]);
			edge *e_C = &(edge_list[e.edge_C]);
			//edge *e_Bl = edge_list + e_B->left_edge;
			edge *e_Br = &(edge_list[e_B->right_edge]);

			u32 iv_new = n + S.size();
			edge e_new = {e_B->id0, e.edge_A, e_B->right_vert, iv_new, e_B->right_vert};
			u32 ie_new = edge_list.size();

			e_A->right_edge = ie_new;
			e_A->right_vert = iv_new;

			e_C->left_vert = iv_new;
			e_C->left_edge = e.edge_B;

			e_B->right_vert = ie_new;
			e_B->right_edge = e.edge_C;

			e_Br->left_edge = ie_new;
			e_Br->left_vert = iv_new;

			//printf("(5) %u\n", e.third_vert);

			edge_list.push_back(e_new);			

			events.push(make_event(&(edge_list[0]), e.edge_B));
			events.push(make_event(&(edge_list[0]), ie_new));
			events.push(make_event(&(edge_list[0]), e.edge_C));
			events.push(make_event(&(edge_list[0]), e.edge_A));
			events.push(make_event(&(edge_list[0]), e_new.right_vert));

			//printf("unimplemented!\n");
		} else {
			printf("(1)[h]: %f\n", e.h);
			printf("(0)\n");
			edge::collapse(&(edge_list[0]), e.edge_B, n + S.size());
			events.push(make_event(&(edge_list[0]), e.edge_A));
			events.push(make_event(&(edge_list[0]), e.edge_C));
			printf("(5)\n");
		}
		commit_event(&(edge_list[0]), e);
		printf("(6)\n");
	}
	printf("(7)\n");
	return S;
}

geom::skeleton_2 geom::make_skeleton_from_convex_2(const Poly& P) {
	u32 n = P.size;
	const u32 INVALID_ID = -1u;
	struct vert {
		u32 origin;
		u32 iv_l; // iv means "index of vertice"
		u32 iv_r;
	};
	std::map<std::pair<u32, u32>, std::pair<u32, u32>> edge_replace; // TODO
	std::vector<vert> LAV(n);
	std::priority_queue<sk_event_2> events;
	std::map<u32, u32> iv_to_iske;
	std::vector<sk_event_2> S;

	for(u32 i = 0; i < n; i++) {
		LAV[i] = {i, (u32)mmod(i - 1, n), (i + 1) % n};
	}
	auto right_iv = [&LAV] (u32 iv) -> u32 {
		return LAV[iv].iv_r;
	};
	auto line_at_v = [&P] (vert v) -> Line {
		return {P[v.origin], P[v.origin + 1]};
	};
	auto add_edge_event = [&LAV, &line_at_v, &events] (u32 iv_2) -> void {
		vert v_2 = LAV[iv_2];
		u32 iv_1 = v_2.iv_l, iv_3 = v_2.iv_r;
		vert v_1 = LAV[iv_1], v_3 = LAV[iv_3];
		auto hQ = bis_inter(line_at_v(v_1), line_at_v(v_2), line_at_v(v_3));
		sk_event_2 e = {
			hQ.first, hQ.second, 
			iv_2, iv_3, INVALID_ID, 
			sk_event_2::EDGE
		};
		// printf("new edge event: [%u %u], %f\n", e.iv_l, e.iv_r, e.h);
		events.push(e);
	};
	auto is_inside_bis_triangle = [](vec2 A, vec2 B, vec2 C, vec2 D, vec2 Q) -> bool {
		vec2 ba = normalize(A - B), cb = normalize(B - C), cd = normalize(D - C), bq = Q - B, cq = Q - C;
		return dot(ba + cb, bq) < 0.f && dot(cb - cd, cq) > 0.f; 
	};
	auto is_inside_bis_triangle_2 = [](Line L1, Line L2, Line L3, vec2 Q) -> bool {
		vec2 a = L1.a, b = L1.b, c = L2.a, d = L2.b, e = L3.a, f = L3.b;
		vec2 ab = b - a, cd = d - c, ef = f - e;
		float lab = len(ab), lcd = len(cd), lef = len(ef);
		float l0 = cross(Q - a, ab / lab), l1 = cross(Q - c, cd / lcd), l2 = cross(Q - e, ef / lef);
		// auto print = [] (vec2 P) -> void {printf("(%f %f) ", P.x, P.y);};
		// printf("points:\n");
		// print(a), print(b), print(c), print(d), print(e), print(f), print(Q);
		// printf("\n(6) lengths: %f %f %f\n", l0, l1, l2);
		return ((l0 > l1) ^ (cross(ab, cd) > 0)) && ((l1 < l2) ^ (cross(cd, ef) > 0));
	};
	auto is_inside_bis_triangle_3 = [](vec2 a, vec2 b, vec2 c, vec2 d, vec2 Q) -> bool {
		vec2 ab = b - a, bc = c - b, cd = d - c;
		// float l0 = cross(Q - a, normalize(ab)), l1 = cross(Q - b, normalize(bc)), l2 = cross(Q - c, normalize(cd));
		// return ((l0 > l1) ^ (cross(ab, bc) > 0)) && ((l1 < l2) ^ (cross(bc, cd) > 0));
		return dot(Q - b, normalize(-ab) - normalize(bc)) < 0.f && dot(Q - c, normalize(cd) + normalize(bc)) < 0.f;
	};

	auto is_valid = [&P, &LAV, &right_iv, &is_inside_bis_triangle_3, &line_at_v, &edge_replace, &events] (sk_event_2 e) -> bool {
		// printf("checking: (%u %u %u, %s, h = %f)\n", e.iv_l, e.iv_r, e.iv_e, e.type == sk_event_2::SPLIT ? "split" : "edge/end", e.h);
		bool def_cond = right_iv(e.iv_l) == e.iv_r && right_iv(e.iv_r) != e.iv_l;
		if(e.type != sk_event_2::SPLIT) {
			return def_cond;
		} else {
			if(right_iv(e.iv_e) == e.iv_e) {
				// printf("next(e.iv_e) = e.iv_e\n");
				return false;
			}
			if(!def_cond) {
				if(auto F = edge_replace.find({e.iv_l, e.iv_r}); F != edge_replace.end()) {
					sk_event_2 el = e, er = e;
					el.iv_r = F->second.first, er.iv_l = F->second.second;
					// printf("changed slice event: (%d-%d), %d -> (%d-%d), (%d-%d)\n", e.iv_l, e.iv_r, e.iv_e, el.iv_l, el.iv_r, er.iv_l, er.iv_r);
					if(el.iv_r != -1u) events.push(el);
					if(er.iv_l != -1u) events.push(er);
				}
				return false;
			} else {
				vert v_s = LAV[e.iv_l], v_sr = LAV[e.iv_r];
				vert v_sl = LAV[v_s.iv_l];
				// bool ans = is_inside_bis_triangle_2(line_at_v(v_sl), line_at_v(v_s), line_at_v(v_sr), e.Q);
				s32 io = v_s.origin;
				bool ans = is_inside_bis_triangle_3(P[io - 1], P[io], P[io + 1], P[io + 2], e.Q);
				// printf("is_valid(%u %u %u): %d\n", e.iv_l, e.iv_r, e.iv_e, ans);
				return ans;
			}
		}
	};
	auto merge = [n, &LAV, &add_edge_event, &S, &iv_to_iske, &edge_replace] (sk_event_2 e) -> void {
		u32 iv_l = e.iv_l, iv_r = e.iv_r;
		vert *v_l = &LAV[iv_l], *v_r = &LAV[iv_r];
		u32 iv_ll = v_l->iv_l, iv_rr = v_r->iv_r;
		vert *v_ll = &LAV[iv_ll], *v_rr = &LAV[iv_rr];
		//make nev vertice
		u32 iv_new = LAV.size();
		vert v_new = {v_r->origin, iv_ll, iv_rr};
		//printf("merge: %u, %u -> %u\n", iv_0, iv_1, iv_new);
		//connect to nbs
		v_ll->iv_r = iv_new;
		v_rr->iv_l = iv_new;
		//isolate merged
		v_l->iv_l = v_l->iv_r = iv_l;
		v_r->iv_l = v_r->iv_r = iv_r;
		bool is_end = iv_ll == iv_rr;
		if(is_end) {
			v_ll->iv_l = v_ll->iv_r = iv_ll;
			v_new.iv_l = v_new.iv_r = iv_new;
			e.iv_e = iv_rr;
			e.type = sk_event_2::END;
		}
		// printf("iv_to_iske[%u] = %lu\n", iv_new, S.size() + n);
		iv_to_iske[iv_new] = S.size() + n;
		LAV.push_back(v_new);
		if(!is_end) {
			add_edge_event(iv_ll);
			add_edge_event(iv_new);
		}

		edge_replace[{iv_ll, iv_l}] = {iv_new, -1u};
		edge_replace[{iv_r, iv_rr}] = {-1u, iv_new};

		S.push_back(e);
		// printf("(1) merged: %u, %u -> %u\n", iv_l, iv_r, iv_new);
	};
	auto slice = [n, &LAV, &add_edge_event, &S, &iv_to_iske, &edge_replace] (sk_event_2 e) -> void {
		u32 iv_s = e.iv_l, iv_sr = e.iv_r, iv_x = e.iv_e;
		vert *v_s = &LAV[iv_s], *v_sr = &LAV[iv_sr], *v_x = &LAV[iv_x];
		u32 iv_xl = v_x->iv_l, iv_xr = v_x->iv_r;
		vert *v_xl = &LAV[iv_xl], *v_xr = &LAV[iv_xr];
		//make new vertices
		u32 iv_n = LAV.size(), iv_nr = iv_n + 1;
		vert v_n = {v_x->origin, iv_s, iv_xr}, v_nr = {v_s->origin, iv_xl, iv_sr};
		//connect to existing
		v_s->iv_r = iv_n, v_xr->iv_l = iv_n;
		v_xl->iv_r = iv_nr, v_sr->iv_l = iv_nr;
		//deleting x
		v_x->iv_l = v_x->iv_r = iv_x;
		//and push (later due to possible reallocation => pointers invalidation)
		// printf("iv_to_iske[%u] = %lu\n", iv_n, S.size() + n);
		// printf("iv_to_iske[%u] = %lu\n", iv_nr, S.size() + n);
		iv_to_iske[iv_n] = S.size() + n;
		iv_to_iske[iv_nr] = S.size() + n;
		LAV.push_back(v_n), LAV.push_back(v_nr);

		// printf("(0) sliced: %u-%u, %u -> %u-%u-%u %u-%u-%u\n", iv_s, iv_sr, iv_x, iv_s, iv_n, iv_xr, iv_xl, iv_nr, iv_sr);
		add_edge_event(iv_s);
		add_edge_event(iv_n);
		add_edge_event(iv_xl);
		add_edge_event(iv_nr);
		edge_replace[{iv_s, iv_sr}] = {iv_n, iv_nr};

		edge_replace[{iv_xl, iv_x}] = {iv_nr, -1u};
		edge_replace[{iv_x, iv_xr}] = {-1u, iv_n};

		S.push_back(e);
	};
	auto add_split_event = [&LAV, &line_at_v, &events] (u32 iv_s, u32 iv_x) -> void {
		vert v_x = LAV[iv_x], v_s = LAV[iv_s];
		u32 iv_sr = v_s.iv_r;
		u32 iv_xl = v_x.iv_l, iv_xr = v_x.iv_r;
		vert v_sr = LAV[iv_sr], v_xl = LAV[iv_xl], v_xr = LAV[iv_xr]; 

		auto hQ = bis_inter(line_at_v(v_xl), line_at_v(v_x), line_at_v(v_s));

		sk_event_2 e = {
			hQ.first, hQ.second,
			iv_s, iv_sr, iv_x,
			sk_event_2::SPLIT
		};
		// printf("new split event: [%u %u %u], %f\n", iv_s, iv_sr, iv_x, e.h);
		events.push(e);
	};
	auto add_initial_split_events = [&P, n, &add_split_event] () -> void {
		if(n < 5) return;
		for(u32 k0 = 0; k0 < n; k0++) {
			if(cross(P[k0] - P[k0 - 1], P[k0 + 1] - P[k0]) <= 0) continue;
			for(u32 o0 = (k0 + 2) % n; o0 != mmod(k0 - 2, n); o0 = (o0 + 1) % n) {
				add_split_event(o0, k0);
			} 
		}
	};
	auto add_initial_edge_events = [n, &add_edge_event] () -> void {
		for(u32 i = 0; i < n; i++) {
			add_edge_event(i);
		}
	};

	add_initial_split_events();
	add_initial_edge_events();

	auto print_LAV = [&LAV] () -> void {
		for(u32 i = 0, n = LAV.size(); i < n; i++) {
			vert v = LAV[i];
			if(v.iv_r == i) continue;
			// printf("[%u<-%u->%u: %u] ", v.iv_l, i, v.iv_r, v.origin);
		}
		// printf("\n");
	};

	while(!events.empty()) {
		sk_event_2 e = events.top();
		events.pop();
		if(!(e.h > 0)) {
			// printf("[%u %u %u]: h = %f < 0\n", e.iv_l, e.iv_r, e.iv_e, e.h);
			continue;
		}
		if(!is_valid(e)) continue;
		if(e.type == sk_event_2::SPLIT) {
			//printf("unimplemented!\n");
			// printf("(3)\n");
			slice(e);
			// S.push_back(e);
		} else {
			// printf("(4)\n");
			merge(e);
			// S.push_back(e);
		}
		print_LAV();
	}
	// printf("(2)\n");
	return {S, iv_to_iske};
}
std::vector<Poly> geom::scale_with_skeleton(const Poly& P, const skeleton_2& S, float h) {
	u32 n = P.size;
	struct vert {
		u32 origin;
		u32 iv_l;
		u32 iv_r;
	};

	std::map<std::pair<u32, u32>, std::pair<u32, u32>> edge_replace;
	std::vector<vert> LAV(n);

	for(u32 i = 0; i < n; i++) {
		LAV[i] = {i, (u32)mmod(i - 1, n), (i + 1) % n};
	}
	auto right_iv = [&LAV] (u32 iv) -> u32 {
		return LAV[iv].iv_r;
	};
	auto line_at_v = [&P] (vert v) -> Line {
		return {P[v.origin], P[v.origin + 1]};
	};
	auto merge = [n, &LAV] (sk_event_2 e) -> void {
		u32 iv_l = e.iv_l, iv_r = e.iv_r;
		vert *v_l = &LAV[iv_l], *v_r = &LAV[iv_r];
		u32 iv_ll = v_l->iv_l, iv_rr = v_r->iv_r;
		vert *v_ll = &LAV[iv_ll], *v_rr = &LAV[iv_rr];

		u32 iv_new = LAV.size();
		vert v_new = {v_r->origin, iv_ll, iv_rr};

		v_ll->iv_r = iv_new;
		v_rr->iv_l = iv_new;

		v_l->iv_l = v_l->iv_r = iv_l;
		v_r->iv_l = v_r->iv_r = iv_r;
		bool is_end = iv_ll == iv_rr;
		if(is_end) {
			v_ll->iv_l = v_ll->iv_r = iv_ll;
			v_new.iv_l = v_new.iv_r = iv_new;
			e.iv_e = iv_rr;
			e.type = sk_event_2::END;
		}

		LAV.push_back(v_new);
	};
	auto slice = [n, &LAV] (sk_event_2 e) -> void {
		u32 iv_s = e.iv_l, iv_sr = e.iv_r, iv_x = e.iv_e;
		vert *v_s = &LAV[iv_s], *v_sr = &LAV[iv_sr], *v_x = &LAV[iv_x];
		u32 iv_xl = v_x->iv_l, iv_xr = v_x->iv_r;
		vert *v_xl = &LAV[iv_xl], *v_xr = &LAV[iv_xr];

		u32 iv_n = LAV.size(), iv_nr = iv_n + 1;
		vert v_n = {v_x->origin, iv_s, iv_xr}, v_nr = {v_s->origin, iv_xl, iv_sr};

		v_s->iv_r = iv_n, v_xr->iv_l = iv_n;
		v_xl->iv_r = iv_nr, v_sr->iv_l = iv_nr;

		v_x->iv_l = v_x->iv_r = iv_x;

		LAV.push_back(v_n), LAV.push_back(v_nr);
	};

	for(auto e: S.events) {
		if(e.h > h) break;
		if(e.type == sk_event_2::SPLIT) {
			slice(e);
		} else {
			merge(e);
		}
	}

	// auto next_id = [&LAV] (u32 i) -> u32 {
	// 	return LAV[i].iv_r;
	// };
	auto line_at_iv = [&P, &LAV] (u32 vi) -> Line {
		u32 io = LAV[vi].origin;
		return {P[io], P[io + 1]};
	};
	auto make_gap = [](Line L1, Line L2, float h) -> vec2 {
		vec2 a = L1.a, b = L1.b, c = L2.a, d = L2.b;
		vec2 ab = b - a, cd = d - c;
		vec2 a_ = a + rrot(normalize(ab)) * h, c_ = c + rrot(normalize(cd)) * h;
		float cr = cross(ab, cd);
		float k = fabsf(cr) < 1.e-7f ? 0.f : -cross(a_ - c_, cd) / cr;
		return a_ + k * ab;
	};

	std::vector<bool> used(LAV.size(), false);
	std::vector<Poly> result;

	for(u32 i = 0; i < LAV.size(); i++) {
		if(used[i]) continue;
		u32 j = i;
		Poly Pi;
		do {
			used[j] = true;
			u32 k = LAV[j].iv_r;
			vec2 p = make_gap(line_at_iv(j), line_at_iv(k), h);
			Pi.add(p);
			j = k;
		} while (j != i);
		if(Pi.size > 2) result.push_back(Pi);
	}
	return result;
}
