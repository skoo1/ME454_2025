// Closed-Chain 2-Link System Simulation in p5.js

let m1 = 0.20, m2 = 0.30;
let l1 = 0.10   // half length of body 1
let l2 = 0.15;  // half length of body 2
let I1 = m1 * (2 * l1) ** 2 / 12;
let I2 = m2 * (2 * l2) ** 2 / 12;
let g = 9.81;

let q = [];      // [x1, y1, theta1, x2, y2, theta2]
let q_dot = [];  // [vx1, vy1, w1, vx2, vy2, w2]
let M;           // mass matrix 6x6

let alpha = 3.0;  // Baumgarte stabilization alpha (velocity stabilization)
let beta = 3.0;  // Baumgarte stabilization beta (position stabilization)

let dt = 0.001;

function setup() {
  createCanvas(600, 600);
  frameRate(60);
  scale(300, -300);
  
  // Initial conditions
  let theta1 = 1.0;
  let theta2 = Math.asin(Math.min(1, Math.max(-1, -2 * l1 * Math.sin(theta1) / (2 * l2))));

  let x1 = l1 * Math.cos(theta1);
  let y1 = l1 * Math.sin(theta1);
  let x2 = x1 + l1 * Math.cos(theta1) + l2 * Math.cos(theta2);
  let y2 = y1 + l1 * Math.sin(theta1) + l2 * Math.sin(theta2);

  q = [x1, y1, theta1, x2, y2, theta2];
  q_dot = [0, 0, 0, 0, 0, 0];
  M = math.diag([m1, m1, I1, m2, m2, I2]);
}

function draw() {
  background(255);
  translate(width / 2, height / 2);
  scale(300, -300);

  let simSteps = Math.floor((deltaTime / 1000) / dt);
  
  for (let i = 0; i < simSteps; i++) {
    step();  // perform high-frequency simulation steps
  }
  
  drawSystem(q);
}

function step() {
  let q_ddot = compute_q_ddot(q, q_dot);
  for (let i = 0; i < 6; i++) {
    q_dot[i] += q_ddot[i] * dt;
    q[i]     += q_dot[i] * dt;
  }
}

function compute_q_ddot(q, q_dot) {
  let J = compute_J(q);
  let J_dot = compute_J_dot(q, q_dot);
  let F_ext = [0, -m1 * g, 0, 0, -m2 * g, 0];
  let JT = math.transpose(J);
  let zero = math.zeros(J.length, J.length);

  let C = compute_constraints(q);  // Compute constraint violations
  let C_dot = math.multiply(J, q_dot);  // Compute constraint velocity
  
  // Baumgarte stabilization: adding correction terms
  let b_top = F_ext;
  let b_bottom = math.multiply(-1, math.multiply(J_dot, q_dot));    // Original equation
  b_bottom = math.add(b_bottom, math.multiply(-2 * alpha, C_dot));  // Baumgarte velocity correction
  b_bottom = math.add(b_bottom, math.multiply(-1 * beta * beta, C));// Baumgarte position correction

  let b = math.concat(b_top, b_bottom);
  let A = math.concat(math.concat(M, JT, 1), math.concat(J, zero, 1), 0);
  let x = math.lusolve(A, b).valueOf().flat();
  return x.slice(0, 6); // q_ddot
}

function compute_J(q) {
  let [x1, y1, th1, x2, y2, th2] = q;
  let c1 = Math.cos(th1), s1 = Math.sin(th1);
  let c2 = Math.cos(th2), s2 = Math.sin(th2);  
  return [
    [1, 0,  l1*s1,   0,   0,   0    ],
    [0, 1, -l1*c1,   0,   0,   0    ],
    [1, 0, -l1*s1,  -1,   0,  -l2*s2],
    [0, 1,  l1*c1,   0,  -1,   l2*c2],
    [0, 0,      0,   0,   1,   l2*c2]
  ];
}

function compute_J_dot(q, q_dot) {
  let [x1, y1, th1, x2, y2, th2] = q;
  let [vx1, vy1, w1, vx2, vy2, w2] = q_dot;
  let c1 = Math.cos(th1), s1 = Math.sin(th1);
  let c2 = Math.cos(th2), s2 = Math.sin(th2); 
  return [
    [0, 0,  l1*w1*c1,  0,  0,  0       ],
    [0, 0,  l1*w1*s1,  0,  0,  0       ],
    [0, 0, -l1*w1*c1,  0,  0, -l2*w2*c2],
    [0, 0, -l1*w1*s1,  0,  0, -l2*w2*s2],
    [0, 0,         0,  0,  0, -l2*w2*s2]
  ];
}

function compute_constraints(q) {
  let [x1, y1, th1, x2, y2, th2] = q;
  let c1 = Math.cos(th1), s1 = Math.sin(th1);
  let c2 = Math.cos(th2), s2 = Math.sin(th2);  
  
  let C1 = x1 - l1 * c1;
  let C2 = y1 - l1 * s1; 
  let C3 = x1 + l1 * c1 - x2 + l2 * c2;
  let C4 = y1 + l1 * s1 - y2 + l2 * s2;
  let C5 = y2 + l2 * s2;
  
  return math.matrix([C1, C2, C3, C4, C5]);  // Return all constraints as a matrix
}

function drawSystem(q) {
  let [x1, y1, th1, x2, y2, th2] = q;

  let dx1 = l1 * Math.cos(th1);
  let dy1 = l1 * Math.sin(th1);
  let x1s = x1 - dx1, y1s = y1 - dy1;
  let x1e = x1 + dx1, y1e = y1 + dy1;

  let dx2 = l2 * Math.cos(th2);
  let dy2 = l2 * Math.sin(th2);
  let x2s = x2 - dx2, y2s = y2 - dy2;
  let x2e = x2 + dx2, y2e = y2 + dy2;

  stroke(0, 100, 255);
  strokeWeight(0.005);
  line(x1s, y1s, x1e, y1e);

  stroke(255, 100, 0);
  line(x2s, y2s, x2e, y2e);
}