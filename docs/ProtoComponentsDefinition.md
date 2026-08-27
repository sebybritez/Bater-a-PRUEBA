# Definición de Componentes en Archivos `.proto` (Webots / Erebus)

Un archivo `.proto` (Prototipo de VRML/Webots) es una plantilla parametrizable que define objetos tridimensionales, robots, sensores y elementos del entorno. Permite instanciar objetos reutilizables dentro de la simulación.

---

## 1. Estructura General de un Archivo `.proto`

Un archivo `.proto` se divide en tres secciones principales:
1. **Encabezado (Header):** Información de versión y etiquetas (`VRML_SIM`, `# tags: static`).
2. **Interfaz de Campos (Proto Interface):** Declaración de parámetros públicos y ocultos (`field` y `hiddenField`).
3. **Cuerpo del Prototipo (Proto Body):** Árbol de nodos 3D, lógica procedimental (Lua) y componentes físicos.

```vrml
#VRML_SIM R2023b utf8
# tags: static

PROTO CustomComponent [
  field SFVec3f    translation  0 0 0
  field SFRotation rotation     0 1 0 0
  field SFString   name         "customComponent"
]
{
  Solid {
    translation IS translation
    rotation IS rotation
    name IS name
  }
}
```

---

## 2. Componentes Principales dentro de un `.proto`

### A. Interfaz y Declaración de Campos (`ProtoInterface`)

Los campos definen las propiedades configurables externamente.

| Componente | Descripción | Ejemplo de Sintaxis |
| :--- | :--- | :--- |
| `field` | Campo público modificable en Webots o supervisor. | `field SFVec3f translation 0 0 0` |
| `hiddenField` | Campo interno no expuesto en la interfaz gráfica. | `hiddenField SFFloat maxVelocity 6.28` |
| `vrmlType` | Tipo de dato del campo (Single Value / Multiple Values). | `SFVec3f`, `SFRotation`, `SFString`, `SFBool`, `SFFloat`, `SFInt32`, `MFNode` |
| `isMapping` | Enlace entre el parámetro del prototipo y el nodo interno. | `translation IS translation` |

---

### B. Bloque Procedimental (`ProceduralLogic`)

Permite ejecutar scripts integrados en **Lua** dentro de bloques `%{ ... }%` para generar código dinámicamente según las variables de la interfaz.

```vrml
%{
  local textureFileName = ""
  if fields.isHarmed.value then
    textureFileName = "\"./textures/victimHarmed.png\""
  else
    textureFileName = "\"./textures/victimUnharmed.png\""
  end
}%
Shape {
  appearance Appearance {
    texture ImageTexture {
      url [ %{= textureFileName }% ]
    }
  }
}
```

---

### C. Nodos Raíz y Estructura 3D (`NodeTree`)

| Nodo Componente | Función Principal |
| :--- | :--- |
| `Robot` | Nodo raíz especial para objetos controlables (agentes). |
| `Solid` | Nodo físico para objetos rígidos con masa y colisiones. |
| `Pose` / `Transform` | Define posición (`translation`), orientación (`rotation`) y escala (`scale`). |
| `Group` | Agrupa múltiples nodos sin transformar coordenadas. |

---

### D. Subcomponentes Físicos y Sensoriales

Dentro de los nodos `children [ ... ]` se definen los siguientes componentes:

1. **Geometría y Aspecto (`Shape` & `Appearance`):**
   * `Shape`: Modela la representación visual.
   * `Appearance` / `PBRAppearance`: Controla materiales (`Material`), colores (`diffuseColor`), rugosidad y texturas (`ImageTexture`).
   * `Geometry`: Formas primitivas (`Box`, `Cylinder`, `Sphere`) o mallas poligonales (`IndexedFaceSet`, `Mesh`).

2. **Propiedades Físicas y Colisiones (`Physics` & `BoundingObject`):**
   * `boundingObject`: Define la malla o forma geométrica simplificada usada para detectar colisiones físicas.
   * `Physics`: Define la masa (`mass`), densidad (`density`) y centro de masa (`centerOfMass`).

3. **Articulaciones y Actuadores (`Joints` & `Actuators`):**
   * `HingeJoint` / `SliderJoint`: Permite rotación o traslación respecto a un eje.
   * `RotationalMotor` / `LinearMotor`: Motores para accionar ruedas o brazos.
   * `PositionSensor`: Mide el ángulo o desplazamiento de la articulación.

4. **Sensores Disponibles (`Sensors`):**
   * `DistanceSensor`: Infrarrojos / ultrasonidos para medir distancias a paredes.
   * `Camera`: Captura de imágenes (RGB o profundidad).
   * `InertialUnit` / `Gyro` / `Compass`: Sensores de orientación, aceleración y brújula.
   * `TouchSensor` / `GPS`: Sensores de contacto táctil y posición absoluta.
   * `Emitter` / `Receiver`: Módulos de comunicación inalámbrica.

---

## 3. Ejemplo de Código Parser en Notación CamelCase (`camelCase`)

El siguiente código de ejemplo escrito en **JavaScript (camelCase)** demuestra cómo leer y procesar la estructura de componentes de un archivo `.proto`:

```javascript
/**
 * Clase para analizar y procesar componentes dentro de un archivo .proto de Webots.
 */
class ProtoComponentParser {
  constructor(protoFilePath) {
    this.protoFilePath = protoFilePath;
    this.declaredFields = new Map();
    this.sensorComponentList = [];
    this.actuatorComponentList = [];
    this.hasPhysics = false;
  }

  /**
   * Lee la interfaz de campos declarados en el prototipo.
   * @param {string} protoContent - Contenido crudo del archivo .proto.
   */
  parseFields(protoContent) {
    const fieldRegex = /(field|hiddenField)\s+([A-Za-z0-9]+)\s+([A-Za-z0-9_]+)\s+([^\n]+)/g;
    let matchResult;

    while ((matchResult = fieldRegex.exec(protoContent)) !== null) {
      const fieldAccessType = matchResult[1];
      const fieldDataType = matchResult[2];
      const fieldName = matchResult[3];
      const fieldDefaultValue = matchResult[4].trim();

      this.declaredFields.set(fieldName, {
        fieldAccessType,
        fieldDataType,
        fieldDefaultValue
      });
    }
  }

  /**
   * Identifica los sensores y actuadores presentes en el cuerpo del prototipo.
   * @param {string} protoContent - Contenido crudo del archivo .proto.
   */
  detectSubComponents(protoContent) {
    const knownSensors = ["DistanceSensor", "Camera", "Gyro", "InertialUnit", "PositionSensor", "GPS"];
    const knownActuators = ["RotationalMotor", "LinearMotor"];

    knownSensors.forEach((sensorName) => {
      if (protoContent.includes(sensorName)) {
        this.sensorComponentList.push(sensorName);
      }
    });

    knownActuators.forEach((actuatorName) => {
      if (protoContent.includes(actuatorName)) {
        this.actuatorComponentList.push(actuatorName);
      }
    });

    if (protoContent.includes("Physics {")) {
      this.hasPhysics = true;
    }
  }

  /**
   * Retorna el resumen analizado en notacion camelCase.
   * @returns {Object} Resumen de componentes detectados.
   */
  getAnalysisSummary() {
    return {
      filePath: this.protoFilePath,
      totalFieldsCount: this.declaredFields.size,
      sensorList: this.sensorComponentList,
      actuatorList: this.actuatorComponentList,
      isPhysicalObject: this.hasPhysics
    };
  }
}

// Ejemplo de uso de la funcion procesadora en camelCase
function analyzeProtoFileExample(rawProtoData) {
  const parserInstance = new ProtoComponentParser("c:/game/protos/customRobot.proto");
  parserInstance.parseFields(rawProtoData);
  parserInstance.detectSubComponents(rawProtoData);

  const analysisOutput = parserInstance.getAnalysisSummary();
  console.log("Componentes procesados con exito:", analysisOutput);
  return analysisOutput;
}
```
