package frc.robot.test.console_output;

public abstract class Output {
    private static final String ANSI_RESET = "\u001B[0m";
    private static final String ANSI_RED = "\u001B[31m";
    private static final String ANSI_GREEN = "\u001B[32m";
    private static final String ANSI_YELLOW = "\u001B[33m";
    private static final String ANSI_BLUE = "\u001B[34m";

    private static String formatRed(String text){
        return null;
    }

    private static String formatGreen(String text){
        return null;
    }

    private static String formatYellow(String text){
        return null;
    }

    private static String formatBlue(String text){
        return null;
    }

    public static void warning(String message){
        System.out.printf("%s: %s \n", formatYellow("[WARNING]"), message);
    }

    public static void sucess(String message){
        System.out.printf("%s: %s \n", formatGreen("[SUCCESS]"), message);
    }

    public static void error(String message){
        System.out.printf("%s: %s \n", formatRed("[ERROR]"), message);
    }

    public static void information(String message){
        System.out.printf("%s: %s \n", formatBlue("[INFORMATION]"), message);
    }
}

