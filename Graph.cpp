#include <iostream>
#define _USE_MATH_DEFINES
#include "GL\freeglut.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>
#include <map>
#include <string>
#include <queue>
#include <vector>
#include <algorithm>


# define M_PI	3.14159265358979323846
# define inf 9999

using namespace std;

const int N = 6; // Розмірність матриці суміжності
const int START = 0; // початкова вершина
const int DISTINATION = 5; // кінцева вершина
int ProgramEnd = 0;
int ALGORITHM;
int order = 0; // порядок обходу


void display();
void timerBFS(int value);
void MousePressed(int button, int state, int x, int y);


class Vertex {

public:
	int n;
	double x;
	double y;
	string color;
	bool visited = false;
	int order;

	// for A* algorithm
	int f;	// f = g + heuristic
	int g;	// dist from start to this vertex
	int from; // predecessor vertex (also for Ford-Falkerson algorithm)

	// Ford-Falkerson algorithm
	int flow_value;

	Vertex() {
		n = 0;
		x = 0.;
		y = 0.;
		color = "white";
		visited = false;
		order = 0;
	
	}

	Vertex(int N, double X, double Y, string COLOR) {
		n = N;
		x = X;
		y = Y;
		color = COLOR;
		order = 0;
	}

	bool operator==(Vertex other) {

		if (this->n == other.n)
			return true;
		else
			return false;

	}

};


class Comparator {
public:
	bool operator()(Vertex& v1, Vertex& v2) // Returns true if t1 is earlier than t2
	{
		return v1.f > v2.f;
	}

};


vector<Vertex> Vertexes;

vector<Vertex> VertexesR;

// *fill the vector of vertexes
// all vertexes have white color
void fillVector(ifstream& input) {

	int n;
	double x, y;
	int k = 0;

	while (input >> n) {

		input.ignore(1);
		input >> x;
		input >> y;
		Vertexes.push_back(Vertex(n, x, y, string("white")));
		VertexesR.push_back(Vertex(n, x, y, string("white")));	// *
		k++;
	}
}

// *show the vector of vertexes
void showVector() {
	for (Vertex v : Vertexes) {
		cout << v.n << ',' << '(' << v.x << ',' << v.y << ')' << ',' << v.color << endl;;
	}
}


class Graph {

private:

	queue<Vertex> Q;
	priority_queue<Vertex, vector<Vertex>, Comparator> PQ;
	priority_queue<Vertex, vector<Vertex>, Comparator> rPQ;

	vector<Vertex> V;	// for Prime algorithm

	vector<int> d;	// distances for Dijkstra algorithm
	vector<int> reverse_d; // for Bidirectional Dijkstra algorithm

	int Flow = 0;

public:

	int matrix[N][N];

	int reverseMatrix[N][N]; // for Bidirectional Dijkstra

	int extraMatrix[N][N];	// Ford-Falkerson
	
	vector<Vertex> Path;
	vector<Vertex> proc;	// for Bidirectional Dijkstra
	vector<Vertex> procR;	// for Bidirectional Dijkstra


	Graph(string edges, string points) {

		ifstream in1(edges);
		fillMatrix(matrix, in1);
		//showMatrix(matrix);

		ifstream in2(points);
		fillVector(in2);
		//showVector();
	}

	void setStart(int i) {
		Vertexes[i].color = "grey";
		Vertexes[i].order = 1;
		order = 2;
		Q.push(Vertexes[i]);

		if(ALGORITHM == 3)
			V.push_back(Vertexes[i]);
		else if (ALGORITHM == 4)
			fillDistances(i);
		else if (ALGORITHM == 5) {
			Vertexes[i].g = 0;
			Vertexes[i].f = heuristic(Vertexes[i], Vertexes[DISTINATION]);
			Vertexes[i].color = "grey";
			PQ.push(Vertexes[i]);

			Vertexes[i].visited = 1;
			Vertexes[i].color = "black";
		}
		else if (ALGORITHM == 6) {
			Vertexes[i].visited = 1;
			Vertexes[i].color = "black";
			Vertexes[i].from = i;
			Vertexes[i].flow_value = inf;
			Path.push_back(Vertexes[i]);
		}
		else if (ALGORITHM == 7) {
			fillReverseMatrix();
			fillInfDistances();
			d[START] = 0;
			reverse_d[DISTINATION] = 0;
		}
		else if (ALGORITHM == 8) {
			fillReverseMatrix();
			fillInfDistances();

			Vertexes[i].g = 0;
			Vertexes[i].f = (heuristic(Vertexes[i], Vertexes[DISTINATION]) + heuristic(Vertexes[i], Vertexes[i])) * 0.5;
			Vertexes[i].color = "black";
			PQ.push(Vertexes[i]);

			VertexesR[DISTINATION].g = 0;
			VertexesR[DISTINATION].f = -Vertexes[i].f;
			VertexesR[DISTINATION].color = "black";
			rPQ.push(VertexesR[DISTINATION]);

			Vertexes[i].visited = 1;
			VertexesR[DISTINATION].visited = 1;
		}
	}

	void BFS(); // пошук в ширину
	void DFS(); // пошук в глибину
	void Prime(); // алгоритм Пріма
	void Dijkstra(); // алгоритм Дейкстри
	void A(int goal); // алгоритм А*
	void FordFalkerson(); // алгоритм Форда-Фалкерсона
	void BidirectionalDijkstra();	// Двонаправлений алгоритм Дейкстри
	void BidirectionalA();	// Двонаправлений алгоритм А*

	void updateBFS() {
		BFS();
	}

	void updateDFS() {
		DFS();
	}

	void updatePrime() {
		Prime();
	}

	void updateDijkstra() {
		Dijkstra();
	}

	void updateA() {
		A(DISTINATION);
	}

	void updateFordFalkerson() {
		FordFalkerson();
	}

	void updateBidirectionalDijkstra() {
		BidirectionalDijkstra();
	}
	void updateBidirectionalA() {
		BidirectionalA();
	}


private:

	void fillMatrix(int matrix[N][N], ifstream& input);
	void showMatrix(int matrix[N][N]);
	void fillDistances(int i);
	void printDistances();
	int heuristic(const Vertex& v1, const Vertex& v2);
	void makePath();
	void fillExtraMatrix();
	int calculatePathMaxFlow();
	void resetVertexes();
	void resetPath();
	void fillReverseMatrix();
	void fillInfDistances();
	int min_dist(int& v);
	int min_distR(int& v);
	void ShortestPath();	// Bidirectional Dijkstra
	void ShortestPathA();	// Bidirectional A*
	void makeForwardPath(int vertex);
	void makeReversePath(int vertex);

};


void Graph::ShortestPath() {

	int distance = inf;
	int best_u;

	vector<Vertex> procUnion = proc;
	procUnion.insert(procUnion.end(), procR.begin(), procR.end());

	for (int i = 0; i < procUnion.size(); i++) {

		if (d[i] + reverse_d[i] < distance) {

			best_u = i;

			distance = d[i] + reverse_d[i];
		}

	}

	Path.insert(Path.end(), proc.begin(), proc.end());
	
	std::reverse(procR.begin(), procR.end());

	Path.insert(Path.end(), procR.begin(), procR.end());

	ProgramEnd = 1;

	return;

}


void Graph::ShortestPathA() {

	std::reverse(proc.begin(), proc.end());

	Path.insert(Path.end(), proc.begin(), proc.end());

	Path.insert(Path.end(), procR.begin(), procR.end());

	ProgramEnd = 1;

	return;
}


int Graph::min_dist(int& v) {

	int min_d = inf;

	for (int i = 0; i < N; i++) {

		if (!Vertexes[i].visited && d[i] < min_d) {

			min_d = d[i];
			v = i;

		}
	}

	return min_d;
}


int Graph::min_distR(int& v) {

	int min_d = inf;

	for (int i = 0; i < N; i++) {

		if (!VertexesR[i].visited && reverse_d[i] < min_d) {

			min_d = reverse_d[i];
			v = i;

		}
	}

	return min_d;
}


void Graph::fillInfDistances() {

	for (int i = 0; i < N; i++) {
		d.push_back(inf);
		reverse_d.push_back(inf);
	}

}


void Graph::fillReverseMatrix() {

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			reverseMatrix[i][j] = matrix[j][i];
		}
	}

}


void Graph::resetVertexes() {

	for (Vertex& v : Vertexes) {
		v.color = "white";
		v.visited = false;
		v.flow_value = 0;
	}
}


void Graph::resetPath() {
	Path.clear();
}


int Graph::calculatePathMaxFlow() {
	// return max flow for current path
	int min_flow_value = inf;

	for (Vertex v : Path) {

		if (v.flow_value < min_flow_value) {
			min_flow_value = v.flow_value;
		}

	}

	return min_flow_value;

}


void Graph::fillExtraMatrix() {

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			extraMatrix[i][j] = matrix[i][j];
		}
	}

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			if (extraMatrix[i][j] < inf && extraMatrix[i][j] > 0) {
				extraMatrix[j][i] = 0;
			}
		}
	}
}


void Graph::printDistances() {

	cout << "\n" << "Dijkstra distances" << endl;

	cout << "Start vertex: " << START << endl;

	for (int i = 0; i < N; i++) {

		cout << START << "->" << i << " = " << d[i] << endl;

	}

}


void Graph::fillDistances(int i) {
	// initialisation of Dijkstra algorithm
	d.resize(N);

	V.push_back(Vertexes[i]);
	Vertexes[i].visited = true;
	Vertexes[i].color = "black";

	for (int v = 0; v < N; v++) {
		if (matrix[i][v] < inf) {
			d[v] = matrix[i][v];
		}
		else
			d[v] = inf;
	}

	d[i] = 0;

}


void Graph::DFS() {

	if (Q.empty()) return;

	Vertex u = Q.front();
	// Q.pop();
	int adjNumber = 0;

	for (int v = u.n; v < N; v++) {

		if (Vertexes[v].color == "white" && matrix[u.n][v] < inf) {

			Q.push(Vertexes[v]);
			Vertexes[v].color = "grey";
			Vertexes[v].order = order;
			order += 1;
			adjNumber++;
			break;

		}

	}

	if (!adjNumber) { 
		Vertexes[u.n].color = "black";
		Q.pop();
	}

}


void Graph::BFS() {

	if (Q.empty()) return;

	Vertex current = Q.front();
	Q.pop();
	Vertexes[current.n].visited = true;

	for (int v = 0; v < N; v++) {

		if (!Vertexes[v].visited && matrix[current.n][v] < inf) {
			Q.push(Vertexes[v]);
			Vertexes[v].color = "grey";
		}

	}

	Vertexes[current.n].color = "black";

}


vector<pair<Vertex, Vertex>> Tree; // ребра в кістяковому дереві
void Graph::Prime() {

	if (V.empty()) return;

	int min_cost = inf;
	int best_vertex = -1;
	int adj = 0;
	Vertex cur;

	for (Vertex current : V) {

		for (int v = 0; v < N; v++) {

			if (matrix[current.n][v] < inf) {

				if (matrix[current.n][v] < min_cost && !Vertexes[v].visited) {
					min_cost = matrix[current.n][v];
					best_vertex = v;
					cur = current;
				}
				adj++;
			}

		}

		Vertexes[current.n].visited = true;

		if (adj == 0 || adj == 1) V.erase(remove(V.begin(), V.end(), current), V.end());

	}

	if (best_vertex != -1)
		Vertexes[best_vertex].visited = true;


	if (best_vertex != -1)
		Tree.push_back(make_pair(Vertexes[cur.n], Vertexes[best_vertex]));
		V.push_back(Vertexes[best_vertex]);

		Vertexes[best_vertex].order = order;
		order += 1;

}


void Graph::Dijkstra() {

	if (V.size() == N) {
		printDistances();
		exit(0);
	}

	int min = inf;
	int w = -1;

	for (int k = 0; k < N; k++) {
		if (!Vertexes[k].visited && d[k] < min) {
			min = d[k];
			w = k;
		}
	}
	
	// end of algorithm
	if (min == inf) {
		printDistances();
		exit(0);
	}


	Vertexes[w].color = "black";
	Vertexes[w].visited = true;
	V.push_back(Vertexes[w]);

	for (int v = 0; v < N; v++) {

		if (!Vertexes[v].visited) {
			if (d[v] >= d[w] + matrix[w][v])
				d[v] = d[w] + matrix[w][v];
		}

	}

}


int Graph::heuristic(const Vertex& v1, const Vertex& v2) {

	// return (int)sqrt(((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y)));
	return int(fabs(v1.x - v2.x) + fabs(v1.y - v2.y));

}


void Graph::makeForwardPath(int vertex) {

	proc.clear();

	int cur = vertex;

	proc.push_back(Vertexes[cur]);

	while (cur != START) {

		int from = Vertexes[cur].from;
		proc.push_back(Vertexes[from]);
		cur = from;

	}

}


void Graph::makeReversePath(int vertex) {

	procR.clear();

	int cur = vertex;

	procR.push_back(VertexesR[cur]);

	while (cur != DISTINATION) {

		int from = VertexesR[cur].from;
		procR.push_back(VertexesR[from]);
		cur = from;

	}

}

void Graph::makePath() {

	int cur = DISTINATION;

	Path.push_back(Vertexes[cur]);

	while (cur != START) {

		int from = Vertexes[cur].from;
		Path.push_back(Vertexes[from]);
		cur = from;

	}

}


void Graph::A(int goal) {

	if (!PQ.empty()) {

		Vertex current = PQ.top();

		if (current.n == DISTINATION) { 
			Vertexes[current.n].color = "black";
			makePath();
			return;
		}

		PQ.pop();
		Vertexes[current.n].visited = true;
		Vertexes[current.n].color = "black";

		for (int i = 0; i < N; i++) {

			if (matrix[current.n][i] < inf && !Vertexes[i].visited) {

				int temp_g = current.g + matrix[current.n][i];

				if (Vertexes[i].color != "grey" || temp_g < Vertexes[i].g) {
					Vertexes[i].g = temp_g;
					Vertexes[i].f = Vertexes[i].g + heuristic(Vertexes[i], Vertexes[DISTINATION]);
					Vertexes[i].from = current.n;
				}
				if (Vertexes[i].color != "grey") {
					Vertexes[i].color = "grey";
					PQ.push(Vertexes[i]);
				}
			}
		}
	}
	else
		return;

}


void Graph::FordFalkerson() {

	// step 1
	Vertex current = Path.back();

	if (current.n == DISTINATION) {
		resetVertexes();
		resetPath();
		setStart(START);
	}
	else {

		// step 2, step 3
		int max_flow = -1;
		int next = -1;

		for (int i = 0; i < N; i++) {

			if (!Vertexes[i].visited) {

				if (extraMatrix[current.n][i] < inf && extraMatrix[current.n][i] > 0) {

					if (extraMatrix[current.n][i] > max_flow) {
						max_flow = extraMatrix[current.n][i];
						next = i;
					}
				}
			}
		}

		if (next != -1) {
			// put a label
			Vertexes[next].from = current.n;
			Vertexes[next].flow_value = max_flow;
		}

		// step 5
		if (next == DISTINATION) {

			Vertexes[current.n].color = "black";

			Vertexes[next].color = "grey";
			Path.push_back(Vertexes[next]);

			int max_flow = calculatePathMaxFlow();
			Flow += max_flow;

			for (int i = 0; i < Path.size() - 1; i++) {
				int v1 = Path[i].n;
				int v2 = Path[i + 1].n;
				extraMatrix[v1][v2] -= max_flow;
				extraMatrix[v2][v1] += max_flow;
			}

		}
		else {

			if (next == -1) {

				if (current.n == START) {
					cout << "\n" << "Max flow : " << Flow << endl;
					exit(0);
				}
				else {
					int from = Vertexes[current.n].from;
					extraMatrix[from][current.n] = inf;
					Path.erase(Path.end() - 1);
					Vertexes[current.n].color = "white";
					return;
				}

			}
			else {
				Vertexes[next].color = "grey";
				Path.push_back(Vertexes[next]);
			}

			Vertexes[current.n].visited = true;
			Vertexes[current.n].color = "black";
		}
	}
}


void Graph::BidirectionalDijkstra() {

	if (ProgramEnd) return;

	int v;	// номер поточної вершини
	int minD = min_dist(v);

	for (int i = 0; i < N; i++) {

		if (matrix[v][i] < inf) {

			if (d[i] > d[v] + matrix[v][i]) {
				d[i] = d[v] + matrix[v][i];
				Vertexes[i].from = v;
			}
		}
	}

	Vertexes[v].visited = true;
	Vertexes[v].color = "black";
	proc.push_back(Vertexes[v]);

	for (const Vertex& w : procR) {
		if (v == w.n) {
			ShortestPath();
		}
	}


	int vr;
	int minDr = min_distR(vr);

	for (int i = 0; i < N; i++) {

		if (reverseMatrix[vr][i] < inf) {

			if (reverse_d[i] > reverse_d[vr] + reverseMatrix[vr][i]) {
				reverse_d[i] = reverse_d[vr] + reverseMatrix[vr][i];
				VertexesR[i].from = vr;
			}
		}
	}

	VertexesR[vr].visited = true;
	VertexesR[vr].color = "black";
	procR.push_back(VertexesR[vr]);

	for (const Vertex& w : proc) {
		if (vr == w.n) {
			ShortestPath();
		}
	}

}


void Graph::BidirectionalA() {

	if (ProgramEnd) return;

	Vertex current = PQ.top();

	PQ.pop();
	Vertexes[current.n].visited = true;
	Vertexes[current.n].color = "black";

	proc.push_back(Vertexes[current.n]);


	for (int i = 0; i < N; i++) {

		if (matrix[current.n][i] < inf && !Vertexes[i].visited) {

			int temp_g = current.g + matrix[current.n][i];

			if (Vertexes[i].color != "grey" || temp_g < Vertexes[i].g) {
				Vertexes[i].g = temp_g;
				Vertexes[i].f = Vertexes[i].g + (heuristic(Vertexes[i], Vertexes[DISTINATION]) + heuristic(Vertexes[i], Vertexes[START]))* 0.5;
				Vertexes[i].from = current.n;
			}
			if (Vertexes[i].color != "grey") {
				Vertexes[i].color = "grey";
				PQ.push(Vertexes[i]);
			}
		}
	}

	for (const Vertex& w : procR) {
		if (current.n == w.n) {
			makeForwardPath(current.n);
			makeReversePath(current.n);
			ShortestPathA();
		}
	}


	Vertex currentR = rPQ.top();

	rPQ.pop();
	VertexesR[currentR.n].visited = true;
	VertexesR[currentR.n].color = "black";

	procR.push_back(VertexesR[currentR.n]);

	for (int i = 0; i < N; i++) {

		if (reverseMatrix[currentR.n][i] < inf && !VertexesR[i].visited) {

			int temp_g = currentR.g + reverseMatrix[currentR.n][i];

			if (VertexesR[i].color != "grey" || temp_g < VertexesR[i].g) {
				VertexesR[i].g = temp_g;
				VertexesR[i].f = VertexesR[i].g - (heuristic(Vertexes[i], Vertexes[DISTINATION]) + heuristic(Vertexes[i], Vertexes[START])) * 0.5;
				VertexesR[i].from = currentR.n;
			}
			if (VertexesR[i].color != "grey") {
				VertexesR[i].color = "grey";
				rPQ.push(VertexesR[i]);
			}
		}
	}

	for (const Vertex& w : proc) {
		if (currentR.n == w.n) {
			makeForwardPath(currentR.n);
			makeReversePath(currentR.n);
			ShortestPathA();
		}
	}


}


void displayText(float x, float y, char* s) {
	int j = strlen(s);
	glRasterPos2f(x, y);
	for (int i = 0; i < j; i++) {
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, s[i]);
	}
}


Graph G("edges.txt", "points_coordinates.txt");

void MousePressed(int button, int state, int x, int y) {

	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		
		if (!ProgramEnd) {
			if (ALGORITHM == 1) G.updateBFS();
			if (ALGORITHM == 2) G.updateDFS();
			if (ALGORITHM == 3) G.updatePrime();
			if (ALGORITHM == 4) G.updateDijkstra();
			if (ALGORITHM == 5) G.updateA();
			if (ALGORITHM == 6) G.updateFordFalkerson();
			if (ALGORITHM == 7) G.updateBidirectionalDijkstra();
			if (ALGORITHM == 8) G.updateBidirectionalA();
		}

		glutPostRedisplay();  // Redraw windows
	}
}


void drawPoint(double x, double y, std::string color) {
	glBegin(GL_POINTS);

	if (color == "white") {
		glColor3f(255, 255, 255);
	}
	else if (color == "grey") {
		glColor3f(0.4, 0.4, 0.4);
	}
	else if (color == "black") {
		glColor3f(0, 0, 0);
	}
	glVertex2d(x, y);
	glEnd();
}


void DrawCircle(float cx, float cy, float r, int num_segments) {
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	// glBegin(GL_LINE_LOOP);
	glBegin(GL_POLYGON);
	glColor3f(0.5f, 0.0f, 0.0f);
	for (int ii = 0; ii < num_segments; ii++) {
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle 
		float x = r * cosf(theta);	//calculate the x component 
		float y = r * sinf(theta);	//calculate the y component 
		glVertex2f(x + cx, y + cy);	//output vertex 
	}
	glEnd();
}


void drawEdge(double x1, double y1, double x2, double y2) {

	glLineWidth(2);
	glBegin(GL_LINES);
	glVertex2d(x1, y1);
	glVertex2d(x2, y2);
	glEnd();
	
	// edges direction
	double x, y;
	if (x2 > x1) x = x2 - (x2 - x1) * 0.12;
	else x = x2 + (x1 - x2) * 0.12;
	if (y2 > y1) y = y2 - (y2 - y1) * 0.12;
	else y = y2 + (y1 - y2) * 0.12;
	
	DrawCircle(x, y, 0.010, 20);

}


void display() {

	char buf[80];

	glClear(GL_COLOR_BUFFER_BIT); // clear buffer

	// draw points
	for (Vertex v : Vertexes) {
		drawPoint(v.x, v.y, v.color);
		if (v.order) {
			glColor3f(0.0f, 1.0f, 0.0f);
			_itoa_s(v.order, buf, 80, 10);
			displayText(v.x - 0.02, v.y - 0.02, buf);
		}
	}

	for (Vertex v : VertexesR) {
		if (v.color == "black") {
			drawPoint(v.x, v.y, v.color);
			if (v.order) {
				glColor3f(0.0f, 1.0f, 0.0f);
				_itoa_s(v.order, buf, 80, 10);
				displayText(v.x - 0.02, v.y - 0.02, buf);
			}
		}
	}

	

	// draw edges and costs
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {

			if (G.matrix[i][j] < inf) {
				glColor3f(0, 0, 0);
				drawEdge(Vertexes[i].x, Vertexes[i].y, Vertexes[j].x, Vertexes[j].y);

				glColor3f(1.0f, 1.0f, 0.0f);
				_itoa_s(G.matrix[i][j], buf, 80, 10);
				displayText((Vertexes[i].x + Vertexes[j].x) * 0.5, (Vertexes[i].y + Vertexes[j].y) * 0.5, buf);
			}
		}
	}

	// optimization for not oriented graph
	/*for (int i = 0; i < N; i++) {
		for (int j = i + 1; j < N; j++) {

			if (G.matrix[i][j] < inf) {
				glColor3f(0, 0, 0);
				drawEdge(Vertexes[i].x, Vertexes[i].y, Vertexes[j].x, Vertexes[j].y);

				glColor3f(1.0f, 1.0f, 0.0f);
				_itoa_s(G.matrix[i][j], buf, 80, 10);
				displayText((Vertexes[i].x + Vertexes[j].x)*0.5, (Vertexes[i].y + Vertexes[j].y) * 0.5, buf);
			}
		}
	}*/

	// draw Prime's tree
	if (ALGORITHM == 3) {
		for (auto e : Tree) {

			glColor3f(1.0f, 0.0f, 1.0f);
			drawEdge(e.first.x, e.first.y, e.second.x, e.second.y);

		}
	}

	// draw Ford-Falkerson path and bidirectional path
	if (ALGORITHM == 5 || ALGORITHM == 7 || ALGORITHM == 8) {
		if (G.Path.size()) {
			for (int i = 0; i < G.Path.size() - 1; i++) {

				glColor3f(0.0f, 1.0f, 1.0f);
				drawEdge(G.Path[i].x, G.Path[i].y, G.Path[i + 1].x, G.Path[i + 1].y);

			}
			ProgramEnd = 1;
		}
	}

	glutSwapBuffers();

}

void timerBFS(int value) {

	display();
	glutTimerFunc(100, timerBFS, 0);
	
}

void Graph::fillMatrix(int matrix[N][N], ifstream& input) {

	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			matrix[i][j] = inf;
		}
	}

	int u, v, cost;

	while (input >> u) {

		input >> v;
		input >> cost;
		matrix[u][v] = cost;
		if (ALGORITHM == 1 || ALGORITHM == 2)
			matrix[v][u] = cost; // not oriented graph
	}

	fillExtraMatrix();

}


void Graph::showMatrix(int matrix[N][N]) {
	for (int i = 0, j = 0; i < 6; i++) {

		for (int j = 0; j < 6; j++) {

			cout << matrix[i][j] << ", ";

		}
		cout << endl;

	}
}


void reshape(int w, int h)
{

	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glClearColor(0.7, 0.7, 0.7, 1);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glOrtho(-1, 1, -1, 1, 0, 0);
}


void printMenu() {

	cout << "Algorithms:" << endl;
	cout << "BFS" << " - 1" << endl;
	cout << "DFS" << " - 2" << endl;
	cout << "Prime" << " - 3" << endl;
	cout << "Dijkstra" << " - 4" << endl;
	cout << "A" << " - 5" << endl;
	cout << "Ford-Falkerson" << " - 6" << endl;
	cout << "BidirectionalDijkstra" << " - 7" << endl;
	cout << "BidirectionalA" << " - 8" << endl;

}


void menu() {

	printMenu();

	int n;
	cout << "Please, enter the algorithm number: ";
	cin >> n;

	ALGORITHM = n;

}


int main(int argc, char** argv) {

	menu();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);

	glutInitWindowSize(800, 600);
	glutCreateWindow("Graph");

	// point settings
	glEnable(GL_POINT_SMOOTH);
	glPointSize(30);

	G.setStart(START);

	glutReshapeFunc(reshape);
	glutDisplayFunc(display);
	glutMouseFunc(MousePressed);
	
	glutMainLoop();

	return 0;

}