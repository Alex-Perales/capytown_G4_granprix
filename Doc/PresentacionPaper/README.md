# Paper IEEE — CapyTown Gran Prix (Grupo 4)

Este repositorio contiene el paper corto en formato IEEE (dos columnas) que documenta
el diseño e implementación del sistema de navegación autónoma para el reto
"CapyTown Gran Prix — El Laberinto del Chaski", listo para compilarse en Overleaf.

## Contenido

- `main.tex` — documento principal (clase `IEEEtran`, opción `conference`).
- `refs.bib` — bibliografía (BibTeX).
- `figuras/` — imágenes usadas por el paper (copias en formato ASCII-safe de `Fotos/`).
- `Fotos/` — fotos originales del equipo (nombres con tildes/ñ, se conservan tal cual).

## Cómo subirlo a Overleaf

1. Entra a [overleaf.com](https://www.overleaf.com) → **New Project → Upload Project**.
2. Comprime esta carpeta (`capytown_G4_s13_paper`) en un `.zip` — puedes excluir la
   carpeta `Fotos/` del zip si quieres (no la usa `main.tex`, solo `figuras/`).
3. Sube el `.zip`. Overleaf detecta `main.tex` automáticamente.
4. Verifica en el menú del proyecto que el **compilador sea `pdfLaTeX`** (por defecto
   lo es) y que el **archivo principal (Main document)** sea `main.tex`.
5. Compila (Recompile). Overleaf ejecuta `pdflatex` → `bibtex` → `pdflatex` ×2
   automáticamente al detectar `\bibliography{refs}`; si la primera compilación no
   muestra las referencias numeradas, dale **Recompile** una segunda vez (es
   normal en LaTeX con BibTeX).

Alternativa sin zip: crea el proyecto en blanco en Overleaf, y arrastra/sube
`main.tex`, `refs.bib` y la carpeta `figuras/` (con sus 7 imágenes) directamente
al panel de archivos izquierdo.

## Estructura del paper

Sigue la misma estructura de un paper IEEE completo (título/autores → resumen →
index terms → secciones I–VII → referencias), adaptada al contenido real del
proyecto:

1. **Introducción** — el reto y su objetivo.
2. **Antecedentes** — nodos reutilizados de Retos Clasificatorios previos
   (`box_detector` de RC-4, flujo de calibración HSV, Split & Merge).
3. **Arquitectura del Sistema Propuesto** — nodos ROS 2, FSM de `maze_solver`,
   ley de control PD, detección de PARE, censo de karpinchus, arbitraje
   cámara-LiDAR, capas de seguridad, métricas.
4. **Configuración Experimental** — plataforma, pista física, parámetros.
5. **Resultados** — evidencia fotográfica de validación funcional por
   componente, y estado honesto de las métricas de competencia (pendientes de
   las corridas oficiales).
6. **Discusión** — decisiones de diseño y limitaciones.
7. **Conclusión** — resumen y trabajo pendiente antes de competir.

Todos los valores numéricos (umbrales, ganancias PD, parámetros HSV, columnas
del CSV de métricas) fueron extraídos directamente del código real en
`capytown_granprix_pkg/` (no inventados), para que el paper sea una descripción
fiel de lo implementado hasta ahora.

## Autores (Grupo 4)

- Henry Tovar Landa (24100514)
- Julia Rojas Estrada (24100478)
- Jordy Diaz Huanca (23101308)
- Daniel León Condori (17100703)
- Alex Perales Maldonado (22200107)

## Pendiente antes de la entrega final

El código aún no ha corrido las dos rondas oficiales de competencia
(`meta_enabled=false`, `long_optima_cm=0.0`, `pare_reales=0` en
`config/granprix_params.yaml`). Cuando se corran esas rondas y se complete
`metricas_granprix.csv`, hay que actualizar la Sección V (Resultados) del
paper con los números reales de `tiempo_s`, `eficiencia`, `colisiones`,
`pare_respetados`, etc.
