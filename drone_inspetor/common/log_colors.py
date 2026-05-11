# log_colors.py
# =================================================================================================
# UTILITÁRIO DE CORES ANSI PARA LOGS NO TERMINAL
# =================================================================================================
# Centraliza todos os códigos ANSI de cores, estilos e prefixos de log do projeto.
# Uso: importar `colorir()` para colorir texto arbitrário, ou `LogPrefix` para usar
# os prefixos padronizados de comunicação entre nós.
#
# Exemplo:
#   from drone_inspetor.common.log_colors import Ansi, colorir, LogPrefix
#
#   # Texto arbitrário colorido (com reset automático no final)
#   self.get_logger().info(colorir("Texto em verde!", Ansi.VERDE, Ansi.NEGRITO))
#
#   # Prefixos padronizados de comunicação entre nós
#   self.get_logger().info(LogPrefix.px4_rx("Modo Offboard ativado"))
#   self.get_logger().info(LogPrefix.px4_tx("Comando ARM enviado"))
# =================================================================================================


class Ansi:
    """Códigos ANSI para cores e estilos de texto no terminal."""

    # --- Reset ---
    RESET = "\033[0m"

    # --- Estilos ---
    NEGRITO = "\033[1m"
    ESCURO = "\033[2m"
    ITALICO = "\033[3m"
    SUBLINHADO = "\033[4m"
    PISCANTE = "\033[5m"
    INVERTIDO = "\033[7m"
    RISCADO = "\033[9m"

    # --- Cores padrão ---
    PRETO = "\033[30m"
    VERMELHO = "\033[31m"
    VERDE = "\033[32m"
    AMARELO = "\033[33m"
    AZUL = "\033[34m"
    MAGENTA = "\033[35m"
    CIANO = "\033[36m"
    BRANCO = "\033[37m"

    # --- Cores claras ---
    CINZA_ESCURO = "\033[90m"
    VERMELHO_CLARO = "\033[91m"
    VERDE_CLARO = "\033[92m"
    AMARELO_CLARO = "\033[93m"
    AZUL_CLARO = "\033[94m"
    MAGENTA_CLARO = "\033[95m"
    CIANO_CLARO = "\033[96m"
    BRANCO_CLARO = "\033[97m"


def colorir(texto: str, *estilos: str) -> str:
    """
    Aplica códigos ANSI ao texto com reset automático no final.

    Args:
        texto: Texto a ser formatado.
        *estilos: Um ou mais códigos ANSI da classe Ansi (ex: Ansi.VERDE, Ansi.NEGRITO).

    Returns:
        String formatada com os estilos aplicados e reset ao final.

    Exemplo:
        colorir("Sucesso!", Ansi.VERDE, Ansi.NEGRITO)
        # Resultado: "\\033[32m\\033[1mSucesso!\\033[0m"
    """
    prefixo = "".join(estilos)
    return f"{prefixo}{texto}{Ansi.RESET}"


class LogPrefix:
    """
    Prefixos padronizados para logs de comunicação entre nós ROS2.

    Cada método retorna uma string completa com:
    - Código ANSI de cor/estilo
    - Rótulo identificando a origem/destino da mensagem
    - A mensagem fornecida
    - Reset automático no final (\\033[0m)
    """

    @staticmethod
    def px4_rx(msg: str) -> str:
        """Mensagem/telemetria recebida DO PX4 (ciano + negrito)."""
        return colorir(f"MENSAGEM PX4 - {msg}", Ansi.CIANO, Ansi.NEGRITO)

    @staticmethod
    def px4_tx(msg: str) -> str:
        """Comando enviado PARA o PX4 (magenta + negrito)."""
        return colorir(f"COMANDO PX4 - {msg}", Ansi.MAGENTA, Ansi.NEGRITO)

    @staticmethod
    def mission_rx(msg: str) -> str:
        """Comando recebido DO Mission Node (azul + negrito)."""
        return colorir(f"COMANDO MISSION RECEBIDO - {msg}", Ansi.AZUL, Ansi.NEGRITO)

    @staticmethod
    def drone_tx(msg: str) -> str:
        """Comando enviado PARA o Drone Node (azul + negrito)."""
        return colorir(f"COMANDO PARA DRONE NODE ENVIADO - {msg}", Ansi.AZUL, Ansi.NEGRITO)
