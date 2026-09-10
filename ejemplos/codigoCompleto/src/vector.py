import math 

class Vector:
    @staticmethod
    def from_tuple(tuple):
        return Vector(tuple[0], tuple[1])
    
    @staticmethod
    def from_angle(angle, length = 1):
        x = math.sin(-angle)
        y = math.cos(angle)
        return Vector(x, -y) * length
    
    def __init__ (self, x, y):
        self.x = x
        self.y = y
        
    def distance_to(self, another_vector):
        v1 = another_vector.x - self.x
        v2 = another_vector.y - self.y
        dist = v1**2 + v2**2
        answer = math.sqrt(dist)
        return answer
    
    def normalized(self):
        x = self.x
        y = self.y
        l = x*x+y*y
        if l != 0:
            l = math.sqrt(l)
            x /= l
            y /= l
        return Vector(x, y)

    def length(self):
        x = self.x
        y = self.y
        return math.sqrt(x*x + y*y)

    def angle(self):
        return math.atan2(-self.x, -self.y)

    def angle_to(self, another_vector):
        a = another_vector.cross(self)
        b = another_vector.dot(self)
        return math.atan2(a, b)
    
    def cross(self, another_vector):
        return self.x * another_vector.y - self.y * another_vector.x
    
    def dot(self, another_vector):
        return self.x * another_vector.x + self.y * another_vector.y
    
    def rotated(self, radians):
        a = self.angle() + radians
        l = self.length()
        v = Vector.from_angle(a)
        result = v * l
        return result
    
    def round(self):
        return Vector(round(self.x), round(self.y))
    
    def floor(self):
        return Vector(math.floor(self.x), math.floor(self.y))
    
    def get_opposite(self):
        return self.rotated(math.pi)

    def __add__(self, vector):
        return Vector(self.x + vector.x, self.y + vector.y)
    
    def __sub__(self, vector):
        return Vector(self.x - vector.x, self.y - vector.y)

    def __mul__(self, scalar):
        return Vector(self.x * scalar, self.y * scalar)
    
    def __rmul__(self, scalar):
        return self*scalar
    
    def __truediv__(self, scalar):
        return Vector(self.x / scalar, self.y / scalar)
    
    def __repr__(self):
        return f"Vector({self.x}, {self.y})"
    
    def __eq__(self, value):
        if value is None: return False
        if not isinstance(value, Vector): return False
        return self.x == value.x and self.y == value.y
    
    def __hash__(self):
        key = (self.x, self.y)
        return hash(key)

Vector.UP = Vector(0, -1)
Vector.DOWN = Vector(0, 1)
Vector.LEFT = Vector(-1, 0)
Vector.RIGHT = Vector(1, 0)
Vector.ONE = Vector(1, 1)
Vector.ZERO = Vector(0, 0)
