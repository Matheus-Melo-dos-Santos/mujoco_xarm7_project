# Simulação MuJoCo — UFACTORY xArm7

Projeto-base para a atividade opcional de simulação física de robôs. A aplicação é uma estação simples de organização de peças sobre uma bancada.

> **Importante sobre os arquivos do robô:** o modelo oficial xArm7 possui malhas 3D que não podem ser reconstruídas a partir do XML sozinho. Por isso, este projeto inclui um script que baixa a pasta oficial `ufactory_xarm7` do MuJoCo Menagerie, preservando os arquivos originais e a licença BSD-3-Clause. Execute `./setup_model.sh` antes de iniciar. O download requer Git e conexão com a internet.

## 1. Componentes da cena

- Manipulador UFACTORY xArm7 (7 juntas rotacionais).
- Bancada, caixa de origem, caixa de destino e peças estáticas.
- Movimento individual das sete juntas.
- Sequência coordenada de configurações articulares: repouso → área A → área B → repouso.

A demonstração não implementa cinemática inversa, agarramento, controle de força ou planejamento de trajetória cartesiana. As configurações A e B são poses articulares de demonstração; não há garantia de que a ponta da ferramenta coincida precisamente com o centro das caixas.

## 2. Requisitos

- Python 3.10 ou superior.
- Git.
- MuJoCo Python `mujoco` 2.3.3 ou superior (a versão instalada via `pip` é recomendada).
- Sistema com ambiente gráfico compatível com o viewer do MuJoCo.

## 3. Instalação

Linux/macOS:

```bash
git clone <URL_DO_SEU_REPOSITORIO>
cd mujoco_xarm7_project

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt

chmod +x setup_model.sh
./setup_model.sh
```

Windows PowerShell:

```powershell
git clone <URL_DO_SEU_REPOSITORIO>
cd mujoco_xarm7_project

py -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
bash setup_model.sh
```

O script de preparação faz um clone esparso do repositório oficial e baixa somente `ufactory_xarm7`. A fonte do modelo é:

<https://github.com/google-deepmind/mujoco_menagerie/tree/main/ufactory_xarm7>

## 4. Executar

A partir da raiz do projeto, com o ambiente virtual ativado:

```bash
python scripts/main.py
```

O visualizador abre e a sequência é executada automaticamente. Para encerrar, feche a janela do viewer.

No macOS, se o viewer passivo não abrir por restrições da plataforma, execute:

```bash
mjpython scripts/main.py
```

### 4.1 - Erro na execução

Nos testes, foi obtido uma falha na hora de encontrar os arquivos .stl. A correção foi alterar o arquivo ```mujoco_xarm7_project/vendor/mujoco_menagerie/ufactory_xarm7/xarm7.xml```, adicionando o assets/*.stl. Para facilitar a reprodução:
 
```python
    <mesh file="assets/link_base.stl"/>
    <mesh file="assets/link1.stl"/>
    <mesh file="assets/link2.stl"/>
    <mesh file="assets/link3.stl"/>
    <mesh file="assets/link4.stl"/>
    <mesh file="assets/link5.stl"/>
    <mesh file="assets/link6.stl"/>
    <mesh file="assets/link7.stl"/>
    <mesh file="assets/end_tool.stl"/>
    <mesh file="assets/base_link.stl"/>
    <mesh file="assets/left_outer_knuckle.stl"/>
    <mesh file="assets/left_finger.stl"/>
    <mesh file="assets/left_inner_knuckle.stl"/>
    <mesh file="assets/right_outer_knuckle.stl"/>
    <mesh file="assets/right_finger.stl"/>
    <mesh file="assets/right_inner_knuckle.stl"/>
```

## 5. O que o código faz

1. Carrega `scene/inspection_cell.xml`.
2. Obtém do próprio `MjModel` os nomes, limites e IDs das sete juntas.
3. Localiza os atuadores `act1` a `act7` definidos pelo modelo oficial.
4. Para cada junta, mantém as demais na posição de repouso e move a junta selecionada entre 35% e 45% de sua faixa permitida.
5. Executa uma sequência coordenada com sete referências simultâneas para a área A, área B e posição de repouso.
6. Avança a simulação com `mujoco.mj_step` e sincroniza o viewer.

Os valores de limites são lidos em tempo de execução de `model.jnt_range`; não são copiados manualmente para o script. O modelo Menagerie define os atuadores de posição do braço como `act1` até `act7`.

## 6. Verificação do modelo

Após `./setup_model.sh`, a estrutura esperada é:

```text
mujoco_xarm7_project/
├── README.md
├── requirements.txt
├── setup_model.sh
├── scene/
│   └── inspection_cell.xml
├── scripts/
│   └── main.py
└── vendor/
    └── mujoco_menagerie/
        └── ufactory_xarm7/
            ├── xarm7.xml
            ├── scene.xml
            ├── README.md
            ├── LICENSE
            └── assets/
```

O modelo oficial declara as juntas `joint1` a `joint7` e os atuadores `act1` a `act7`. Os limites particulares são consultados no XML compilado em tempo de execução. O modelo requer MuJoCo 2.3.3 ou posterior.

## 7. Uso de IA

- **Ferramentas utilizadas:** ChatGPT.
- **Como foram utilizadas:** Geração dos scripts e escolha do modelo.
- **Exemplo de contribuição útil:** Geração rapida de um modelo pronto para validação.
- **Exemplo de erro ou limitação:** Não define limites válidos, apresentando colisão. Mesmo tendo acesso aos modelos prontos e valores definidos no medelo, não foi capaz de realizar pedidos complexos, pedir para posicionar o objeto de fim de curso numa posição específica do espaço, fazendo ajustes apenas nas juntas do robô - ou seja, necessário implementar manualmente f(x,y,z) = (r1,r2,r3,r4,r5,r6,r7). 
- **Verificação humana:** Ajuste das colisão na verificação das juntas.

## 8. Referências

- Google DeepMind. **MuJoCo Menagerie — UFACTORY xArm7**. <https://github.com/google-deepmind/mujoco_menagerie/tree/main/ufactory_xarm7>
- Google DeepMind. **MuJoCo Python documentation**. <https://mujoco.readthedocs.io/en/stable/python.html>
- O modelo xArm7 do Menagerie é distribuído sob BSD-3-Clause. Consulte `vendor/mujoco_menagerie/ufactory_xarm7/LICENSE` após executar o script de preparação.

## 9. Limitações conhecidas

- O projeto depende de conexão com a internet uma única vez para obter o modelo e as malhas oficiais.
- As poses A e B são referências articulares demonstrativas, não resultados de cinemática inversa.
- A cena não possui sensores, esteira animada ou agarramento de peças.
- Antes da gravação final, observe a simulação e ajuste as poses se necessário para evitar colisões ou posições visualmente inadequadas.
