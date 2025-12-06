import matplotlib.pyplot as plt

def read_csv(filename):
    xs=[]
    ys=[]
    thetas_deg=[]
    with open(filename,"r") as f:
        next(f)  # skip header
        for line in f:
            line=line.strip()
            if not line:
                continue
            x,y,theta=line.split(",")
            xs.append(float(x))
            ys.append(float(y))
            thetas_deg.append(float(theta))
    return xs,ys,thetas_deg


sx,sy,sth=read_csv("Straight_path.csv")
cx,cy,cth=read_csv("Circular_path.csv")  # dt small, smooth curve
rx,ry,rth=read_csv("Rotated_path.csv")


plt.figure(figsize=(10,4))  # one big image

# LEFT GRAPH : Robot Paths
plt.subplot(1,2,1)
plt.plot(sx,sy,label="Straight")
plt.plot(cx,cy,label="Circular")
plt.plot(rx,ry,label="Rotated",marker="o")

plt.title("Week 1 - Robot Paths")
plt.xlabel("x (m)")
plt.ylabel("y (m)")
plt.grid(True)
plt.axis("equal")
plt.legend()


# RIGHT GRAPH : Theta change (Rotation Test)
steps=list(range(len(rth)))
plt.subplot(1,2,2)
plt.plot(steps,rth)
plt.title("Rotation Test - Heading vs Step")
plt.xlabel("Step")
plt.ylabel("theta (deg)")
plt.grid(True)

plt.savefig("week1_path_plot.png", dpi=300)
plt.show()
