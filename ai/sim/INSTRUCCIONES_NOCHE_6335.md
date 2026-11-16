# Entrenamiento nocturno — Fase 6.3.3.5

1. Descomprime el ZIP en WSL, por ejemplo `unzip IA_Enjambre_Fase6_3_3_5_Entrenamiento_Nocturno.zip -d ~/noche6335`.
2. Entra en `~/noche6335/IA_Enjambre_Fase5/sim` (el nombre interno es historico; puedes renombrarlo).
3. Activa tu entorno virtual de Python, donde esten instalados PyBullet, NumPy y PyTorch.
4. Ejecuta `export SWARM_IK_STRIDE=4` y `python -u entrenar_noche_6335.py --hours 8`.
5. Para desconectarte de la terminal sin interrumpir: `nohup python -u entrenar_noche_6335.py --hours 8 > noche_consola.log 2>&1 < /dev/null &` (no lances ambas ordenes).
6. Por la manana consulta `out/noche6335/resumen.json`, `out/noche6335/noche.log`, `out/noche6335/mappo/evaluation.csv` y `out/noche6335/mappo/checkpoint_latest.pt`.

## Que hace

- BC con 30.000 muestras sinteticas, si hay anclas reutiliza las previas; si no, recopila 60 episodios.
- Evaluacion fisica de profesor, actor BC y mezclas en 1, 2 y 6 brazos.
- Solo inicializa MAPPO desde BC si el actor puro supera un 25 % de exito en la prueba de un brazo.
- Entrena MAPPO siguiendo su curriculum original (300/300/400), con checkpoints por episodio y evaluaciones cada 25.
- Limita la ejecucion al presupuesto de horas especificado. Puede interrumpir un episodio en curso al agotar el tiempo, pero conserva el ultimo checkpoint.
- Reejecutar el mismo comando continua MAPPO desde el checkpoint. Las evaluaciones previas no se repiten.

## Advertencias

- WSL y el ordenador deben permanecer encendidos; desactiva suspension de Windows. `nohup` no evita la suspension.
- No se garantiza una tasa de exito concreta; los datos sinteticos no sustituyen trayectorias fisicas.
- `SWARM_IK_STRIDE=4` acelera, pero debe validarse su precision con `stride=1`.
- Para evitar sobreescribir entrenamientos existentes, usa una carpeta `--out` distinta de las anteriores.
- No mezcles los archivos `sim` de versiones distintas. El ZIP contiene una copia autocontenida del proyecto.
